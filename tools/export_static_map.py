"""Project registered, deskewed LiDAR scans into a Nav2 occupancy map offline."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import label
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def floor_plane(points):
    hist, edges=np.histogram(points[:,2],bins=np.arange(-1.5,.1,.01))
    seed=(edges[:-1]+edges[1:])[np.argmax(hist)]/2
    p=points[np.abs(points[:,2]-seed)<.09]
    # Balance repeated dense floor samples using a 5cm voxel grid.
    _,idx=np.unique(np.floor(p/.05).astype(np.int32),axis=0,return_index=True);p=p[idx]
    a=np.column_stack([p[:,:2],np.ones(len(p))]);b=p[:,2]
    coef=np.linalg.lstsq(a,b,rcond=None)[0]
    for _ in range(8):
        residual=b-a@coef; w=np.minimum(1.,.015/np.maximum(np.abs(residual),1e-6))
        coef=np.linalg.lstsq(a*w[:,None],b*w,rcond=None)[0]
    residual=b-a@coef
    normal=np.array([-coef[0],-coef[1],1.]);normal/=np.linalg.norm(normal)
    x=np.array([1.,0.,0.]);x-=normal*np.dot(x,normal);x/=np.linalg.norm(x)
    rot=np.vstack([x,np.cross(normal,x),normal])
    return coef,rot,float(np.median(np.abs(residual)))


def binary_pcd(path, points):
    header=f'''# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA binary
'''
    with path.open('wb') as f:f.write(header.encode());f.write(np.asarray(points,dtype='<f4').tobytes())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('registered');p.add_argument('output')
    p.add_argument('--resolution',type=float,default=.05)
    p.add_argument('--min-height',type=float,default=.10)
    p.add_argument('--max-height',type=float,default=.60)
    p.add_argument('--max-range',type=float,default=8.)
    p.add_argument('--scan-step',type=int,default=2)
    p.add_argument('--crop-to-walls',action='store_true',help='Crop output bounds to the largest observed wall component')
    p.add_argument('--align-walls',action='store_true',help='Align dominant near-axis walls using registered points before rasterization')
    p.add_argument('--crop-margin',type=float,default=.25)
    args=p.parse_args()
    if args.resolution<=0 or args.scan_step<1 or args.min_height>=args.max_height:raise ValueError('Invalid projection parameters')
    root=Path(args.registered);out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    files=sorted((root/'scans').glob('*.npz'))
    if len(files)<10:raise ValueError('Too few registered scans')
    sample=[]
    for f in files[::max(1,len(files)//150)]:
        d=np.load(f);q=d['points'];dist=np.linalg.norm(q[:,:2]-d['pose'][1:3],axis=1)
        sample.append(q[(dist>.4)&(dist<3.)][::2])
    coeff,rot,floor_mad=floor_plane(np.concatenate(sample))
    # LiDAR position relative to IMU, from the exact FAST-LIO run parameters.
    params=yaml.safe_load((root/'parameters.yaml').read_text())['/**']['ros__parameters']
    ext=np.asarray(params['mapping']['extrinsic_T'])
    first=np.load(files[0])['pose']
    shift=first[1:4]+Rotation.from_quat(first[4:8]).apply(ext)
    alignment_degrees=0.
    if args.align_walls:
        # Search a small global yaw correction using concentration of wall
        # projections. No maze-specific coordinates or wall editing.
        wall=[]
        for f in files[::max(1,len(files)//100)]:
            q=np.load(f)['points'];v=(q-shift)@rot.T
            height=q@rot[2]-coeff[2]/np.sqrt(1+coeff[0]**2+coeff[1]**2)
            wall.append(v[(height>=args.min_height)&(height<=args.max_height),:2])
        wall=np.concatenate(wall)
        _,index=np.unique(np.floor(wall/.025).astype(int),axis=0,return_index=True)
        wall=wall[index]
        scores=[];angles=np.linspace(-5.,5.,401)
        for degrees in angles:
            r=Rotation.from_euler('z',degrees,degrees=True).as_matrix()[:2,:2]
            v=wall@r.T
            score=0.
            for axis in range(2):
                bins=np.floor(v[:,axis]/.025).astype(int);bins-=bins.min()
                hist=np.bincount(bins).astype(float)
                score+=np.dot(hist,hist)
            scores.append(score)
        alignment_degrees=float(angles[np.argmax(scores)])
        if abs(alignment_degrees)>=5.:raise ValueError('Wall alignment reached search boundary; inspect map')
        rot=Rotation.from_euler('z',alignment_degrees,degrees=True).as_matrix()@rot
    def project(q):
        q=np.asarray(q);v=(q-shift)@rot.T
        v[...,2]=(q@rot[2])-coeff[2]/np.sqrt(1+coeff[0]**2+coeff[1]**2)
        return v
    trajectory=np.loadtxt(root/'trajectory.csv',delimiter=',',skiprows=1)
    tr=project(trajectory[:,1:4])
    yaw=(Rotation.from_matrix(rot)*Rotation.from_quat(trajectory[:,4:8])).as_euler('xyz')[:,2]
    all_points=[];frames=[]
    for i,f in enumerate(files):
        d=np.load(f);q=project(d['points']);pose=d['pose'];origin=project(pose[1:4]+Rotation.from_quat(pose[4:8]).apply(ext))
        dist=np.linalg.norm(q[:,:2]-origin[:2],axis=1)
        mask=(dist>.28)&(dist<args.max_range)&(q[:,2]>-.06)&(q[:,2]<2.)
        q=q[mask]
        all_points.append(q[::2])
        if i%args.scan_step==0:
            obstacles=q[(q[:,2]>=args.min_height)&(q[:,2]<=args.max_height)]
            if not len(obstacles):continue
            delta=obstacles[:,:2]-origin[:2];distance=np.linalg.norm(delta,axis=1)
            bins=np.floor((np.arctan2(delta[:,1],delta[:,0])+np.pi)/(np.pi/360)).astype(int)
            # Nearest endpoint in each 0.5-degree bin blocks rays at walls.
            order=np.argsort(distance);_,idx=np.unique(bins[order],return_index=True)
            frames.append((origin[:2],obstacles[order[idx],:2]))
    points=np.concatenate(all_points).astype(np.float32)
    _,idx=np.unique(np.floor(points/.05).astype(np.int32),axis=0,return_index=True)
    points=points[idx];binary_pcd(out/'maze.pcd',points)
    endpoints=np.concatenate([p for _,p in frames]);bounds=np.vstack([endpoints,tr[:,:2]])
    lo=np.floor((bounds.min(0)-.5)/args.resolution)*args.resolution
    hi=np.ceil((bounds.max(0)+.5)/args.resolution)*args.resolution
    size=np.ceil((hi-lo)/args.resolution).astype(int)
    width,height=size;total=width*height
    odds=np.zeros(total,dtype=np.float32);hits=np.zeros(total,dtype=np.int32);free=np.zeros(total,dtype=np.int32)
    def cells(q):return np.floor((q-lo)/args.resolution).astype(int)
    for origin,endpoints in frames:
        hit=cells(endpoints);hit=hit[:,1]*width+hit[:,0];hit=np.unique(hit)
        delta=endpoints-origin
        steps=np.ceil(np.linalg.norm(delta,axis=1)/args.resolution*2).astype(int)
        k=np.arange(max(steps))
        valid=k[None,:]<steps[:,None]
        t=k[None,:]/steps[:,None]
        ray=origin+delta[:,None,:]*t[:,:,None]
        grid=cells(ray[valid]);flat=np.unique(grid[:,1]*width+grid[:,0]);flat=np.setdiff1d(flat,hit,assume_unique=True)
        odds[flat]-=.25;free[flat]+=1
        odds[hit]+=.85;hits[hit]+=1
        np.clip(odds,-5.,5.,out=odds)
    # Traversed footprint provides genuine free observations near the sensor.
    for pos in tr[:, :2]:
        xy=cells(pos);r=int(np.ceil(.15/args.resolution))
        for dx in range(-r,r+1):
            for dy in range(-r,r+1):
                if (dx*dx+dy*dy)*args.resolution**2>.15**2:continue
                x,y=xy+[dx,dy]
                if 0<=x<width and 0<=y<height:free[y*width+x]+=1;odds[y*width+x]=min(odds[y*width+x],-1.)
    occupied=(odds>1.)&(hits>=3)
    empty=(free>0)&(odds<=0.)
    image=np.full(total,205,dtype=np.uint8);image[empty]=254;image[occupied]=0
    image=image.reshape(height,width)
    full_size=[int(width),int(height)]
    if args.crop_to_walls:
        components,count=label(image==0,structure=np.ones((3,3)))
        sizes=np.bincount(components.ravel());sizes[0]=0
        significant=components==np.argmax(sizes)
        yy,xx=np.where(significant)
        if not len(xx):raise ValueError('No substantial wall components to crop')
        margin=int(np.ceil(args.crop_margin/args.resolution))
        x0=max(0,int(xx.min())-margin);x1=min(width,int(xx.max())+margin+1)
        y0=max(0,int(yy.min())-margin);y1=min(height,int(yy.max())+margin+1)
        old_shape=(height,width)
        odds=odds.reshape(old_shape)[y0:y1,x0:x1].ravel()
        hits=hits.reshape(old_shape)[y0:y1,x0:x1].ravel()
        free=free.reshape(old_shape)[y0:y1,x0:x1].ravel()
        image=image[y0:y1,x0:x1];height,width=image.shape
        lo+=np.array([x0,y0])*args.resolution;hi=lo+np.array([width,height])*args.resolution
        occupied=image.ravel()==0;empty=image.ravel()==254
    with (out/'maze.pgm').open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode());f.write(np.flipud(image).tobytes())
    yaml_map={'image':'maze.pgm','mode':'trinary','resolution':args.resolution,
              'origin':[float(lo[0]),float(lo[1]),0.], 'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (out/'maze.yaml').write_text(yaml.safe_dump(yaml_map,sort_keys=False))
    plt.imsave(out/'maze.png',np.flipud(image),cmap='gray',vmin=0,vmax=255)
    fig,ax=plt.subplots(figsize=(10,9));ax.imshow(image,origin='lower',cmap='gray',vmin=0,vmax=255,extent=[lo[0],hi[0],lo[1],hi[1]],interpolation='nearest')
    ax.plot(tr[:,0],tr[:,1],color='#00a9b4',lw=1.,label='LiDAR/IMU trajectory')
    ax.scatter(tr[0,0],tr[0,1],c='limegreen',s=50,label='Start');ax.scatter(tr[-1,0],tr[-1,1],c='orange',s=50,label='End')
    ax.set_xlabel('x, m');ax.set_ylabel('y, m');ax.legend(loc='lower center',bbox_to_anchor=(.5,1.02),ncol=3,fontsize=9);ax.set_aspect('equal');fig.tight_layout();fig.savefig(out/'preview.png',dpi=160);plt.close(fig)
    np.savez_compressed(out/'occupancy_evidence.npz',log_odds=odds.reshape(height,width),hit_scans=hits.reshape(height,width),free_scans=free.reshape(height,width))
    np.savetxt(out/'trajectory.csv',np.column_stack([trajectory[:,0],tr,yaw]),delimiter=',',header='timestamp,x,y,z,yaw',comments='')
    distance=float(np.linalg.norm(np.diff(tr,axis=0),axis=1).sum())
    report={'wall_alignment_degrees':alignment_degrees,'registered_scans':len(files),'crop_to_walls':args.crop_to_walls,'crop_margin_m':args.crop_margin,'crop_policy':'largest observed occupied component bounds; preserve every cell inside crop','full_size_cells':full_size,'projection_scans':len(frames),'cloud_voxels':len(points),'floor_plane_raw':coeff.tolist(),
            'floor_residual_median_abs_m':floor_mad,'raw_to_map_rotation':rot.tolist(),'raw_xy_shift':shift.tolist(),
            'resolution_m':args.resolution,'size_cells':[int(width),int(height)],'origin':yaml_map['origin'],
            'height_band_m':[args.min_height,args.max_height],'range_m':args.max_range,
            'occupied_cells':int(occupied.sum()),'free_cells':int(empty.sum()),'unknown_cells':int((image==205).sum()),
            'trajectory_length_m':distance,'start_end_distance_m':float(np.linalg.norm(tr[-1,:2]-tr[0,:2])),
            'loop_closure_applied':False,'physical_localization_verified':False}
    (out/'quality.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
