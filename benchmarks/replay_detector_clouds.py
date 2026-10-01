#!/usr/bin/env python3
"""Replay saved map-frame clouds through the current C++ detector core.

Truth positions only label the output offline; they never enter detect().
Requires a host C++17 compiler, no ROS installation.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CPP = r'''
#include "hsl_perception/detector.hpp"
#include <iostream>
#include <iomanip>
#include <array>
int main(int argc,char ** argv) {
  const double height=std::stod(argv[1]);
  hsl_perception::Grid grid;
  std::cin>>grid.resolution>>grid.width>>grid.height>>grid.origin_x>>grid.origin_y;
  for (int i=0;i<grid.width*grid.height;++i) {int v;std::cin>>v;grid.data.push_back(v);}
  std::array<std::optional<hsl_perception::Position>,2> previous;
  std::array<double,2> last{-1e20,-1e20};
  int observer; double time,x,y; size_t count;
  while (std::cin>>observer>>time>>x>>y>>count) {
    std::vector<hsl_perception::Point> points(count);
    for (auto & p:points) {std::cin>>p.x>>p.y>>p.z;}
    const auto prior=time-last[observer]>=0 && time-last[observer]<=1.0 ? previous[observer] : std::nullopt;
    const auto detection=hsl_perception::detect(points,grid,{x,y},prior,height);
    if (detection) {
      previous[observer]=detection->centre;last[observer]=time;
      std::cout<<std::setprecision(12)<<detection->centre.x<<" "<<detection->centre.y<<" "
        <<detection->hits<<" "<<detection->extent<<"\n";
    } else {std::cout<<"none\n";}
  }
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--height', type=float, default=0.46)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.height) or not 0.08 <= args.height <= 0.60:
        parser.error("height must be within [0.08, 0.60]")
    data = json.loads(args.recording.read_text())
    grid = data['replay_grid']
    lines = [' '.join(str(grid[k]) for k in ('resolution','width','height','origin_x','origin_y')),
             ' '.join(map(str,grid['data']))]
    clouds = data['cloud_samples']
    for sample in clouds:
        lines.append(' '.join(map(str, [int(sample['observer']=='peer_scan'),
            sample['received_sim_s'], *sample['own_xy'], len(sample['points'])])))
        lines.extend(' '.join(map(str,point)) for point in sample['points'])
    with tempfile.TemporaryDirectory(prefix='hsl-detector-replay-') as directory:
        cpp, binary = Path(directory)/'replay.cpp', Path(directory)/'replay'
        cpp.write_text(CPP)
        subprocess.run(['g++','-O2','-std=c++17','-I',str(ROOT/'src/hsl_perception/include'),
                        str(cpp),'-o',str(binary)],check=True,timeout=60)
        result = subprocess.run([str(binary),str(args.height)],input='\n'.join(lines)+'\n',
                                text=True,capture_output=True,check=True,timeout=60)
    outputs = result.stdout.splitlines()
    if len(outputs) != len(clouds):
        raise RuntimeError('C++ replay did not return one output per cloud')
    rows=[]
    counts=collections.Counter()
    for sample, output in zip(clouds, outputs):
        row = dict(observer=sample['observer'], stamp_sim_s=sample['stamp_sim_s'])
        if output=='none':
            row['label']='no_detection'
        else:
            x,y,hits,extent=map(float,output.split())
            peer_error=math.dist((x,y),sample['peer_truth_xy'])
            box_error=math.dist((x,y),data['fixture_map_xy'])
            label=('ambiguous' if peer_error<.4 and box_error<.4 else
                   'peer_near' if peer_error<.4 else 'fixture_near' if box_error<.4 else 'other')
            row.update(label=label,centre=[x,y],hits=int(hits),extent=extent,
                       peer_error_m=peer_error,fixture_error_m=box_error)
        counts[row['label']]+=1
        rows.append(row)
    report=dict(recording=str(args.recording),height=args.height,cloud_count=len(clouds),
                counts=dict(counts),rows=rows,
                scope='offline replay; proximity labels are not exhaustive semantic truth or visibility recall')
    if args.output:
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='rows'},indent=2))


if __name__=='__main__':
    main()
