#!/usr/bin/env python3
"""Prepare real bag clouds and time the native C++ detector, excluding I/O/TF/filter timing."""
import argparse,json,os,struct,subprocess,sys,time,hashlib
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_perception'),str(ROOT/'src/hsl_real')]

def prepare(session,out):
 import yaml
 from audit_real_bags import messages,xyz,transform,stamp
 from rclpy.duration import Duration
 from rclpy.time import Time
 from tf2_ros import Buffer
 from hsl_real.lidar_filter import filter_cloud
 files=sorted((session/'bag').glob('*.mcap'))
 if not files:raise RuntimeError('missing MCAP')
 tf=Buffer(cache_time=Duration(seconds=6000));grid=None
 for file in files:
  for topic,m,_ in messages(file,['/tf','/tf_static','/map']):
   if topic=='/map':grid={'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'width':m.info.width,'height':m.info.height,'data':list(m.data)}
   else:
    for item in m.transforms:(tf.set_transform_static if topic=='/tf_static' else tf.set_transform)(item,'bag')
 if grid is None:raise RuntimeError('recorded map missing')
 params=yaml.safe_load((ROOT/'config/lidar_filter.yaml').read_text())['real_lidar_filter']['ros__parameters']
 chunks=[];samples=[];missing=[];raw_count=0
 for file in files:
  for _,m,_ in messages(file,['/livox/lidar']):
   raw_count+=1;t=stamp(m.header.stamp)
   try:trans=tf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp))
   except Exception:missing.append(t);continue
   # Shared preprocessing, deliberately not timed. No XYZ/temporal decimation.
   filtered,_=filter_cloud(m,params);points=transform(xyz(filtered,0),trans)
   points=points[np.isfinite(points).all(axis=1)].astype(np.float32)
   v=trans.transform.translation;samples.append({'t':t,'sensor':[v.x,v.y,v.z],'chunk':len(chunks)});chunks.append(points)
 out.mkdir(parents=True,exist_ok=True)
 meta={'session':str(session),'raw_clouds':raw_count,'frames':len(chunks),'missing_tf_stamps':missing,'grid':grid,'samples':samples,'point_limit':0,'period_s':0,'lidar_filter_parameters':params,'bag_files':[{'name':f.name,'bytes':f.stat().st_size} for f in files]}
 (out/'input.json').write_text(json.dumps(meta)+'\n')
 np.savez_compressed(out/'input.npz',points=np.concatenate(chunks),offsets=np.cumsum([0]+[len(c) for c in chunks]))
 with (out/'input.bin').open('wb') as f:
  f.write(b'HSLBEN01');f.write(struct.pack('<dddII',grid['resolution'],*grid['origin'],grid['width'],grid['height']));f.write(np.array(grid['data'],dtype='<i4').tobytes());f.write(struct.pack('<I',len(chunks)))
  for s,p in zip(samples,chunks):f.write(struct.pack('<ddddI',s['t'],*s['sensor'],len(p)));f.write(p.astype('<f4').tobytes())
 print(json.dumps({k:meta[k] for k in ('session','raw_clouds','frames','missing_tf_stamps')},indent=2))

def run(directory,binary,repeats):
 affinity=os.sched_getaffinity(0);cpu=min(affinity);os.sched_setaffinity(0,{cpu})
 timings=[]
 for i in range(repeats):
  rows=json.loads(subprocess.check_output([str(binary.resolve()),str((directory/'input.bin').resolve())],text=True))
  timings.extend(r['ms'] for r in rows)
 a=np.array(timings)
 report={'scope':'native detector core only; excludes raw decode, filtering, TF, DDS and output', 'cpu_affinity':cpu,'repeats':repeats,'mean_ms':float(a.mean()),'median_ms':float(np.median(a)),'p95_ms':float(np.percentile(a,95)),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
 np.savez_compressed(directory/'native-timings.npz',cpp_ms=timings)
 (directory/'native-report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepare-session',type=Path);p.add_argument('--output',type=Path);p.add_argument('--input',type=Path);p.add_argument('--binary',type=Path);p.add_argument('--repeats',type=int,default=5);a=p.parse_args()
 if a.prepare_session:
  if not a.output:p.error('--output required')
  prepare(a.prepare_session,a.output)
 else:
  if not a.input or not a.binary or a.repeats<1:p.error('--input/--binary and positive repeats required')
  run(a.input,a.binary,a.repeats)
