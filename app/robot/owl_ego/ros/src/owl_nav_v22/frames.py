"""Transport adapters; planner/public poses remain in the odometry world."""
import math
import numpy as np

class Frames:
    def __init__(self, profile):
        if profile not in ('standard_enu', 'owl_vendor_world'):
            raise ValueError('unknown mavros_frame_profile: '+str(profile))
        self.profile=profile
        self.vendor=profile=='owl_vendor_world'

    def world_velocity(self, quaternion_rotation, velocity):
        # Vendor local_position.cpp publishes WORLD linear twist despite base_link child.
        v=np.asarray(velocity,dtype=float)
        return v.copy() if self.vendor else quaternion_rotation@v

    def yaw_rate(self, rotation, angular):
        # Vendor swaps body roll/pitch rate components while retaining yaw rate.
        wx,wy,wz=angular
        if self.vendor:wx,wy=-wy,wx
        roll=math.atan2(rotation[2,1],rotation[2,2])
        cos_pitch=math.sqrt(max(0.,1-float(rotation[2,0])**2))
        if cos_pitch<.1:raise ValueError('yaw rate undefined near vertical attitude')
        return (math.sin(roll)*wy+math.cos(roll)*wz)/cos_pitch

    def setpoint(self, position, velocity, acceleration, yaw):
        if not self.vendor:return position,velocity,acceleration,yaw
        def rotate(v):return np.array([-v[1],v[0],v[2]],dtype=float)
        return rotate(position),rotate(velocity),rotate(acceleration),(yaw+math.pi/2+math.pi)%(2*math.pi)-math.pi

class Alignment:
    """Compare stamped vendor map and LIO references to control-world odometry."""
    def __init__(self):
        from collections import deque
        self.samples={k:deque(maxlen=40) for k in ('map','lio')}
        self.odoms=deque(maxlen=90)
        self.checked={k:float('-inf') for k in self.samples}
        self.good_at={k:float('-inf') for k in self.samples}
        self.error='waiting for map/LIO alignment samples'

    def add(self, kind, stamp, xyz, yaw):
        if not np.isfinite([stamp,*xyz,yaw]).all():
            raise ValueError('nonfinite '+kind+' reference')
        self.samples[kind].append((stamp,np.asarray(xyz),yaw))

    def check(self, stamp, xyz, yaw, now):
        if not self.odoms or stamp>self.odoms[-1][0]:
            self.odoms.append((stamp,np.asarray(xyz),yaw,now))
        elif stamp<self.odoms[-1][0]:
            self.odoms.clear()
            self.checked={k:float('-inf') for k in self.samples}
            self.good_at={k:float('-inf') for k in self.samples}
            self.odoms.append((stamp,np.asarray(xyz),yaw,now))
        for kind,samples in self.samples.items():
            pair=None
            for r in reversed(samples):
                if r[0]<=self.checked[kind]:continue
                o=min(self.odoms,key=lambda a:abs(a[0]-r[0]))
                # map and odom are sibling messages from the same MAVROS callback.
                # Never compare different attitude times, even within 50 ms.
                tolerance=1e-6 if kind=='map' else .05
                if abs(o[0]-r[0])<=tolerance and now-o[3]<.5:
                    pair=(r,o);break
            if pair is None:continue
            (t,p,y),(_,position,heading,received_at)=pair
            self.checked[kind]=t
            if kind=='map':p=np.array([p[1],-p[0],p[2]]);y-=math.pi/2
            dist=float(np.linalg.norm(position-p))
            angle=abs((heading-y+math.pi)%(2*math.pi)-math.pi)
            if dist>(.03 if kind=='map' else .25) or angle>math.radians(2 if kind=='map' else 10):
                self.good_at[kind]=float('-inf')
                self.error='%s/world mismatch: %.3f m, %.2f deg'%(kind,dist,math.degrees(angle))
                raise ValueError(self.error)
            self.good_at[kind]=received_at
        if self.ready(now):self.error=None

    def ready(self, now):
        return all(now-t<.5 for t in self.good_at.values())
