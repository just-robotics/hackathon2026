"""Sensor-only classification of ignorable 15 x 15 x 40 cm boxes.

Known wall cells are always retained. Partial/merged/low/large clusters remain
obstacles. Neither fixture coordinates nor opponent ground truth are inputs.
"""
import numpy as np
from hsl_perception.geometry import StaticBackground
from hsl_perception.geometry import cluster_xy, split_clusters


def rectangle_fit(xy):
    """Fit visible faces of a 15 cm square, allowing only one visible face.

    A minimum-area rectangle rotates a two-face L by 45 degrees and
    incorrectly turns a square into a triangular/diagonal fragment.
    """
    # 2-degree sampling has < 6 mm dimension error for a 15 cm box.
    angles=np.linspace(0,np.pi/2,46)
    axes=np.stack((np.cos(angles),np.sin(angles)),axis=1)
    x=xy@axes.T; y=xy@np.stack((-np.sin(angles),np.cos(angles)),axis=1).T
    lowx,highx=np.percentile(x,[5,95],axis=0); lowy,highy=np.percentile(y,[5,95],axis=0)
    spans=np.stack((highx-lowx,highy-lowy),axis=1)
    edge=np.minimum.reduce((abs(x-lowx),abs(highx-x),abs(y-lowy),abs(highy-y)))
    fractions=np.mean(edge<=.025,axis=0)
    best=int(np.argmin(np.abs(np.max(spans,axis=1)-.15)+.15*(1-fractions)))
    return np.sort(spans[best]),float(fractions[best])


def classify_cluster(points):
    if len(points)<12:return False,{'reason':'sparse','points':len(points)}
    low,top=np.percentile(points[:,2],[5,98])
    occupied_z=len(np.unique(np.floor(points[(points[:,2]>=.08)&(points[:,2]<=.36),2]/.04)))
    middle_share=float(np.mean((points[:,2]>=.23)&(points[:,2]<=.34)))
    # Height outside both birth and cached-identity rules cannot be ignored.
    # Skip the 46-angle surface fit for these definitely retained objects.
    # Axis bounds are diagnostic only; never feed them into a box decision.
    if top < .25 or top > .46:
        spans=np.percentile(points[:,:2],95,axis=0)-np.percentile(points[:,:2],5,axis=0)
        return False,{'reason':'retain','points':len(points),
            'dimensions':np.sort(spans).tolist(),'dimensions_source':'axis_bounds',
            'height':float(top),'low_p5':float(low),'vertical_bins':occupied_z,
            'middle_share':middle_share,'center':np.mean(points[:,:2],axis=0).tolist()}
    dimensions,edge_fraction=rectangle_fit(points[:,:2])
    narrow=(.12<=dimensions[1]<=.19 and dimensions[0]<=.18 and .32<=top<=.46
            and low<=.26 and occupied_z>=3 and edge_fraction>=.80 and middle_share>=.20)
    return narrow,{'reason':'small_box' if narrow else 'retain','points':len(points),
                   'dimensions':dimensions.tolist(),'height':float(top),'low_p5':float(low),
                   'vertical_bins':occupied_z,'middle_share':middle_share,'edge_fraction':edge_fraction,
                   'center':np.mean(points[:,:2],axis=0).tolist()}


class SmallBoxFilter:
    def __init__(self, confirmation_window=1.2):
        self.confirmation_window=float(confirmation_window)
        if not np.isfinite(self.confirmation_window) or self.confirmation_window<=0:
            raise ValueError("confirmation_window must be positive and finite")
        self.static=None
        self.ignored=np.empty((0,3))
        self.boxes=[]
        self.pending=[]
        self.last_stamp=None

    def set_grid(self,resolution,origin,width,height,data):
        self.static=StaticBackground(resolution,origin,width,height,data,margin=.08)

    def filter(self,points,stamp=None,protected_centers=()):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        if stamp is not None:
            if self.last_stamp is not None and stamp < self.last_stamp-.5:
                self.boxes=[];self.pending=[]
            self.last_stamp=stamp
            self.boxes=[(center,last) for center,last in self.boxes if 0<=stamp-last<=8.]
            self.pending=[item for item in self.pending if 0<=stamp-item[1]<=self.confirmation_window]
        keep=np.ones(len(points),dtype=bool)
        candidates=np.isfinite(points).all(axis=1)&(points[:,2]>=.06)&(points[:,2]<=.65)
        if self.static is None:
            self.ignored=np.empty((0,3))
            return points,{'ignored_points':0,'clusters':[],'ready':False}
        candidates &= self.static.foreground(points)
        indices=np.flatnonzero(candidates)
        labels,count=cluster_xy(points[indices,:2],.08)
        diagnostics=[]
        for cluster_indices in split_clusters(indices.reshape(-1,1),labels,count):
            idx=cluster_indices.ravel()
            # A short view of an already identified robot can look like a box.
            # Protect its current observed body before geometric/cache rules.
            if any(np.any(np.linalg.norm(points[idx,:2]-center,axis=1)<=.25)
                   for center in protected_centers):
                # A robot observation contradicts any previous box identity
                # for this same fragment. Do not revive that identity when
                # the measured track later expires or becomes occluded.
                center=np.mean(points[idx,:2],axis=0)
                self.boxes=[item for item in self.boxes if np.linalg.norm(center-item[0])>=.20]
                self.pending=[item for item in self.pending if np.linalg.norm(center-item[0])>=.20]
                diagnostics.append({'reason':'tracked_robot','points':len(idx)})
                continue
            ignored,diag=classify_cluster(points[idx])
            if stamp is not None and diag['reason']=='retain' and (
                    diag['height']<.25 or diag['height']>.46 or
                    diag['dimensions'][1]>.25):
                # A fuller view can disprove an earlier narrow fragment.
                # Keeping its identity would let the next sparse view erase
                # this broad/low/tall object again for up to eight seconds.
                center=np.array(diag['center'])
                lower,upper=np.percentile(points[idx,:2],[1,99],axis=0)
                def contradicted(item):
                    old=item[0]
                    return (np.linalg.norm(center-old)<.12 or
                            np.all(old>=lower-.025) and np.all(old<=upper+.025))
                old_count=len(self.boxes)+len(self.pending)
                self.boxes=[item for item in self.boxes if not contradicted(item)]
                self.pending=[item for item in self.pending if not contradicted(item)]
                invalidated=old_count-len(self.boxes)-len(self.pending)
                if invalidated:
                    diag['invalidated_box_hypotheses']=invalidated
            if ignored and stamp is not None:
                center=np.array(diag['center'])
                # One uncertain scan must not establish an eight-second box
                # identity. Require three distinct, spatially consistent views.
                known=any(np.linalg.norm(center-old)<.12 for old,last in self.boxes)
                if not known:
                    matches=[item for item in self.pending if np.linalg.norm(center-item[0])<.12]
                    previous=max(matches,key=lambda item:item[2],default=None)
                    views=[] if previous is None else [
                        view for view in previous[3] if 0<=stamp-view[0]<=self.confirmation_window]
                    if not views or stamp>views[-1][0]:
                        views.append((stamp,points[idx].copy()))
                    views=views[-7:]
                    hits=len(views)
                    self.pending=[item for item in self.pending if np.linalg.norm(center-item[0])>=.12]
                    self.pending.append((center,stamp,hits,views))
                    ignored=False
                    if hits>=3:
                        # Independently plausible fragments must also describe
                        # one consistent object in map coordinates. A drifting
                        # body fragment must not establish an eight-second box.
                        ignored,combined=classify_cluster(np.concatenate([v[1] for v in views]))
                        diag['combined_shape']=combined
                    if not ignored:
                        diag.update(reason=('small_box_tentative' if hits<3 else
                                            'small_box_inconsistent_views'),confirmations=hits)
            if not ignored and stamp is not None and diag['reason']=='sparse' and len(idx):
                # At contact the .25 m self mask leaves only a few box returns.
                # They may maintain an established identity, never open one.
                cluster=points[idx]
                center=np.mean(cluster[:,:2],axis=0)
                ignored=(np.max(np.ptp(cluster[:,:2],axis=0))<=.25
                    and np.max(cluster[:,2])<=.46
                    and any(np.linalg.norm(center-old)<.20 for old,last in self.boxes))
                if ignored:
                    diag.update(reason='small_box_cached_sparse',center=center.tolist())
            if not ignored and stamp is not None and diag['reason']=='retain':
                # A partial view may maintain an already identified box, but
                # cannot initialize one. Wide/low/robot-body clusters are kept.
                center=np.array(diag['center'])
                ignored=(diag['dimensions'][1]<=.25 and .25<=diag['height']<=.46
                    and diag['middle_share']>=.18
                    and any(np.linalg.norm(center-old)<.12 for old,last in self.boxes))
                if ignored:diag['reason']='small_box_cached'
            diagnostics.append(diag)
            if ignored:
                keep[idx]=False
                if stamp is not None:
                    center=np.array(diag['center'])
                    self.boxes=[(old,last) for old,last in self.boxes if np.linalg.norm(center-old)>=.12]
                    self.boxes.append((center,stamp))
        self.ignored=points[~keep]
        return points[keep],{'ignored_points':int(np.sum(~keep)),'clusters':diagnostics,'ready':True}
