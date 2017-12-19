import numpy as np
from rllab.envs.box2d.parser import find_body
from rllab.core.serializable import Serializable
from rllab.envs.box2d.box2d_env import Box2DEnv
from rllab.misc import autoargs
from rllab.misc.overrides import overrides
import time

class ArmEnv(Box2DEnv, Serializable):
    @autoargs.inherit(Box2DEnv.__init__)
    def __init__(self, *args, **kwargs):
        kwargs["frame_skip"] = kwargs.get("frame_skip", 2)
        if kwargs.get("template_args", {}).get("noise", False):
            self.link_len = (np.random.rand()-0.5) + 1
        else:
            self.link_len = 1
        kwargs["template_args"] = kwargs.get("template_args", {})
        kwargs["template_args"]["link_len"] = self.link_len

        #self.dt = .1    # refresh rate
        #self.on_goal = 0

        super(ArmEnv, self).__init__(
            self.model_path("arm_env.xml.mako"),
            *args, **kwargs
        )
        self.arm1 = find_body(self.world, "arm1")
        self.arm2 = find_body(self.world, "arm2")
        self.point = find_body(self.world, "point")
        self.vertices=np.array([0.0,0.0])
        Serializable.__init__(self, *args, **kwargs)


    @overrides
    def reset(self):
        self._set_state(self.initial_state)
        self._invalidate_state_caches()
        bounds = np.array([
            [-np.pi,np.pi],
            [-np.pi,np.pi]
        ])
        low, high = bounds
        aarm1,aarm2 = np.random.uniform(low, high)
        self.arm1.angle = aarm1
        self.arm2.angle = aarm2
        return self.get_current_obs()


    @overrides
    def compute_reward(self, action):
        yield
        self.vertices[0] ,self.vertices[1]=self.get_tip_pos()
        #dist1 = [(0.1 - self.arm1.vertices[0]), (-0.5 - self.arm1.vertices[1])]
        dist2 = [(1 - self.vertices[0]), (0.6 - self.vertices[1])]
        yield -np.sqrt(dist2[0]**2+dist2[1]**2)



    @overrides
    def is_current_done(self):
        a=((1 - 0.05 < self.vertices[0] < 1 + 0.05) and \
        (0.6 - 0.05 < self.vertices[1] < 0.6 + 0.05))
        if a: print(a)
        return a

    def get_tip_pos(self):
        cur_center_pos = self.arm2.position
        cur_angle = self.arm2.angle
        cur_pos = (
            cur_center_pos[0] - self.link_len*np.sin(cur_angle),
            cur_center_pos[1] + self.link_len*np.cos(cur_angle)
        )
        return cur_pos




if __name__=="__main__":
    ar=ArmEnv()
    #ca.printinfo()
    #print (type(ar.extra_data.states))
    a=[0,0]
    b=[0,0]
    ar.render()
    time.sleep(2)
    while True:

        #print ("vertices:",ar.vertices)
        ar.step(np.array([30]))
        #print("arm2:", ar.arm2.position)
        #print ("tip:",ar.get_tip_pos())
        ar.render()
        a[0],a[1]=ar.get_tip_pos()
        b[0],b[1]=ar.arm2.position
        print (ar.is_current_done())

        #print (((abs(a[0]-b[0]))**2+ (abs(a[1]-b[1])**2))**0.5)

        time.sleep(0.2)
