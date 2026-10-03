#!/usr/bin/env python3
"""Read recorded transport/TF readiness; no DDS, drivers, or filter timing."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import numpy as np
from audit_real_bags import messages, stamp


def audit(session):
    from tf2_ros import Buffer
    from rclpy.duration import Duration
    from rclpy.time import Time
    tf=Buffer(cache_time=Duration(seconds=6000))
    topics=['/tf','/tf_static','/livox/lidar','/sensing/lidar/points_filtered',
            '/navigation/scan','/navigation/self','/sensing/lidar/filter_diagnostics']
    rows={t:[] for t in topics}; failures=Counter(); pending=deque(maxlen=8)
    queue_delays=[]; dropped=0; filter_rows=[]; immediate=0
    for file in sorted((session/'bag').glob('*.mcap')):
        for topic,msg,t in messages(file,topics):
            if topic in ('/tf','/tf_static'):
                for item in msg.transforms:
                    (tf.set_transform_static if topic=='/tf_static' else tf.set_transform)(item,'bag')
            elif topic=='/sensing/lidar/filter_diagnostics':
                filter_rows.append(json.loads(msg.data))
            else:
                rows[topic].append((t,stamp(msg.header.stamp)))
                if topic=='/sensing/lidar/points_filtered':
                    try:tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp));immediate+=1
                    except Exception as e:failures[str(e).split('.')[0]]+=1
                    if len(pending)==pending.maxlen:dropped+=1
                    pending.append((t,msg.header))
            while pending:
                arrival,h=pending[0]
                # Bound by observation freshness, do not manufacture a new stamp.
                if t-arrival>.8:
                    pending.popleft();dropped+=1;continue
                try:tf.lookup_transform('map',h.frame_id,Time.from_msg(h.stamp))
                except Exception:break
                queue_delays.append(t-arrival);pending.popleft()
    result={'session':session.name,'stages':{},'tf_at_filtered_arrival':immediate,
            'tf_failures':dict(failures),'queued_available':len(queue_delays),'queue_dropped':dropped,
            'queue_transport_delay_s':dict(zip(['median','p95','max'],np.percentile(queue_delays,[50,95,100]).tolist())) if queue_delays else None}
    for topic,values in rows.items():
        if not values:continue
        a=np.array(values);gaps=np.diff(a[:,0]);ages=a[:,0]-a[:,1]
        result['stages'][topic]={'count':len(a),'arrival_gap_p95_s':float(np.percentile(gaps,95)) if len(gaps) else None,
            'arrival_gap_max_s':float(gaps.max()) if len(gaps) else None,'header_age_p95_s':float(np.percentile(ages,95))}
    if filter_rows:
        sums=Counter()
        for row in filter_rows:
            for k,v in row.items():
                if isinstance(v,(int,float)) and k.endswith('points'):sums[k]+=v
        result['filter_points']=dict(sums)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import rclpy
    rclpy.init();reports=[]
    for session in sorted(a.root.iterdir()):
        if not (session/'bag').is_dir():continue
        result=audit(session);reports.append(result);print(json.dumps(result),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(reports,indent=2)+'\n');rclpy.shutdown()

if __name__=='__main__':main()
