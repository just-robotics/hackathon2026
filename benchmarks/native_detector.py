"""Offline subprocess transport to the C++ replay executable; no detection logic."""
import os,shutil,subprocess,json
from pathlib import Path
from types import SimpleNamespace
import numpy as np

class NativeReplay:
    def __init__(self, profile='real', **parameters):
        self.child=None
        candidates=[os.environ.get('HSL_DETECTOR_REPLAY',''),shutil.which('detector_replay') or '']
        candidates += [str(Path(prefix)/'lib/hsl_perception_cpp/detector_replay') for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(':') if prefix]
        self.binary=next((x for x in candidates if x and Path(x).is_file()),None)
        if not self.binary:raise RuntimeError('C++ detector_replay missing: source ROS workspace or set HSL_DETECTOR_REPLAY')
        self.profile=profile;self.parameters=parameters;self.child=None
    def step(self,points,sensor,background,stamp):
        if self.child is None:
            command=[self.binary,self.profile]
            for key,value in self.parameters.items():command.extend(['--'+key.replace('_','-'),str(value)])
            self.child=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            g=background
            self.child.stdin.write(f'{g.resolution} {g.origin[0]} {g.origin[1]} {g.width} {g.height} '+' '.join(map(str,g.grid.ravel()))+'\n');self.child.stdin.flush()
        points=np.asarray(points,dtype=float).reshape(-1,3);points=points[np.isfinite(points).all(axis=1)]
        fields=[stamp,*sensor,len(points)]
        self.child.stdin.write(' '.join(format(v,'.17g') for v in fields)+'\n'+' '.join(format(v,'.17g') for v in points.ravel())+'\n');self.child.stdin.flush()
        line=self.child.stdout.readline()
        if not line:raise RuntimeError(f'C++ detector stopped ({self.child.poll()})')
        d=json.loads(line);track=None
        if d['track_xy'] is not None:track=SimpleNamespace(mean=np.array([*d['track_xy'],*d['velocity']]),last_update=d['last_update'],hits=d['hits'])
        return track,{k:d[k] for k in ('foreground_points','clusters','strong_candidates','tracks','candidates','rejections','weak_reasons')}
    def close(self):
        if self.child is not None:
            self.child.stdin.close()
            try:self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:self.child.kill();self.child.wait()
            self.child=None
    def __del__(self):
        self.close()
