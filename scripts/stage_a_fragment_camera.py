"""Disposable image-fixture profiles. No physical clearance/timing authority."""
import math
import numpy as np
OVERLAP='authored_oblique_overlap_fixture'
SEPARATED='separated_workpieces'
def require(ok,reason):
    if not ok:raise ValueError(reason)

def separation(ids,identities):
    require(ids.dtype==np.uint32 and ids.shape==(256,256) and len(identities)==2 and
        len(set(identities))==2 and set(np.unique(ids))=={0,*identities},'unexpected/missing image identities')
    counts={};bounds={}
    for identity in identities:
        y,x=np.where(ids==identity);require(len(x)>0,'missing workpiece')
        counts[str(identity)]=len(x);bounds[str(identity)]=[int(x.min()),int(y.min()),int(x.max()),int(y.max())]
    ordered=sorted(identities,key=lambda i:bounds[str(i)][0]);a,b=[bounds[str(i)] for i in ordered]
    return dict(pixel_counts=counts,bounds=bounds,blank_columns=b[0]-a[2]-1)

def validate_profile(report,ids):
    profile=report.get('camera_profile');acquisition=report.get('acquisition',{})
    require(profile in (OVERLAP,SEPARATED),'unknown camera profile')
    require(acquisition.get('camera_profile',OVERLAP)==profile,'stale acquisition camera profile')
    identities=[v['id'] for v in report['inventory']]
    if profile==OVERLAP:
        w=report.get('occlusion_witness',{})
        require(report.get('occlusion_verified') is True and w.get('near_id') in identities and
            w.get('far_id') in identities and w['near_id']!=w['far_id'] and
            type(w.get('x')) is int and type(w.get('y')) is int and
            0<=w['x']<ids.shape[1] and 0<=w['y']<ids.shape[0] and
            int(ids[w['y'],w['x']])==w.get('captured_id')==w['near_id'],'native occlusion witness missing/incorrect')
    else:
        actual=separation(ids,identities)
        require(actual==report.get('separation_witness'),'incorrect separated-view witness')
        require(min(actual['pixel_counts'].values())>=1000 and actual['blank_columns']>=12,'separated-view fixture size/gap insufficient')
    return profile

def design(root):
    """Analytic pinhole corner framing from authored BOXes, never planning poses."""
    world=root.find('world');rows=[];f=128/math.tan(math.pi/6)
    for model in world.findall('model'):
        pose=list(map(float,model.findtext('pose').split()));require(len(pose)==6 and pose[3:5]==[0,0],'unsupported authored camera-design rotation')
        links=model.findall('link');require(len(links)==1 and links[0].find('pose') is None,'unsupported link transform')
        visual=links[0].find('visual');require(visual.find('pose') is None,'unsupported visual transform')
        size=list(map(float,visual.findtext('geometry/box/size').split()));require(size==[.025]*3,'authored 25mm BOX required')
        corners=[]
        for sx in (-.0125,.0125):
            for sy in (-.0125,.0125):
                for sz in (-.0125,.0125):
                    x=pose[0]+math.cos(pose[5])*sx-math.sin(pose[5])*sy
                    y=pose[1]+math.sin(pose[5])*sx+math.cos(pose[5])*sy
                    z=pose[2]+sz;depth=y+.357
                    require(.01<depth<2,'outside camera clip range')
                    corners.append([128+f*(x-.4)/depth,128-f*(z-.0125)/depth])
        v=np.asarray(corners);bounds=[float(v[:,0].min()),float(v[:,1].min()),float(v[:,0].max()),float(v[:,1].max())]
        require(0<bounds[0]<bounds[2]<256 and 0<bounds[1]<bounds[3]<256,'authored cube outside image')
        rows.append(dict(authored_model=model.get('name'),projected_bounds=bounds))
    require(len(rows)==2,'exactly two authored cubes required');rows.sort(key=lambda r:r['projected_bounds'][0])
    gap=rows[1]['projected_bounds'][0]-rows[0]['projected_bounds'][2]
    require(gap>=12,'authored projected separation insufficient')
    return dict(profile=SEPARATED,position=[.4,-.357,.0125],look_at=[.4,-.217,.0125],up=[0,0,1],
        fovy_radians=math.pi/3,near=.01,far=2,aspect=1,source='authored BOX corner pinhole framing only',
        workpieces=rows,projected_gap_pixels=gap,scope='preflight framing diagnostic; measured raster counts required')
