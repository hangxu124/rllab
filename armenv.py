import numpy as np
from rllab.envs.box2d.parser import find_body
import re
from rllab.core.serializable import Serializable
from rllab.envs.box2d.box2d_env import Box2DEnv
from rllab.misc import autoargs
from rllab.misc.overrides import overrides

'''
class ArmEnv(Box2DEnv, Serializable):
    @autoargs.inherit(Box2DEnv.__init__)
    def __init__(self, *args, **kwargs):
        # make sure mdp-level step is 100ms long
        kwargs["frame_skip"] = kwargs.get("frame_skip", 2)
        if kwargs.get("template_args", {}).get("noise", False):
            self.link_len = (np.random.rand() - 0.5) + 1
        else:
            self.link_len = 1
        kwargs["template_args"] = kwargs.get("template_args", {})
        kwargs["template_args"]["link_len"] = self.link_len
        super(ArmEnv, self).__init__(
            self.model_path("arm_env.xml.mako"),
            *args, **kwargs
        )
        self.arm1 = find_body(self.world, "arm1")
        self.arm2 = find_body(self.world, "arm2")
        Serializable.__init__(self, *args, **kwargs)



    def printinfo(self):
        #print (self.get_current_obs())
        #print (self.world.bodies)
        a=str(self.arm2.fixtures)
        match1=re.search(r"vertices:",a).span()
        m1=match1[1]
        ver=a[m1+2:]
        match2=re.search(r"]" , ver).span()
        m2=match2[0]
        ver=ver[:m2]
        ver1=ver.replace("(","")
        ver2=ver1.replace(")","")
        ma=ver2.split(", ")

        x=(float(ma[2])+float(ma[4]))*0.5+self.arm2.position[0]
        y=(float(ma[3])+float(ma[5]))*0.5+self.arm2.position[1]
        print (x,y)


        f=open("info.txt",'w')
        f.write(str(self.arm2))
        #print(self.track.position)

'''




import numpy as np
from rllab.misc.mako_utils import compute_rect_vertices
from rllab.envs.box2d.parser import find_body
from rllab.core.serializable import Serializable
from rllab.envs.box2d.box2d_env import Box2DEnv
from rllab.misc import autoargs
from rllab.misc.overrides import overrides

class ArmEnv(Box2DEnv, Serializable):
    @autoargs.inherit(Box2DEnv.__init__)
    def __init__(self, *args, **kwargs):

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
            [-np.pi/2,np.pi/2],
            [-np.pi/2,np.pi/2]
        ])
        low, high = bounds
        aarm1,aarm2 = np.random.uniform(low, high)
        self.arm1.angle = aarm1
        self.arm2.angle = aarm2
        return self.get_current_obs()


    @overrides
    def compute_reward(self, action):
        yield
        self.vertices[0] ,self.vertices[1]=self.calculate_pos()
        #dist1 = [(0.1 - self.arm1.vertices[0]), (-0.5 - self.arm1.vertices[1])]
        dist2 = [(1 - self.vertices[0]), (0.6 - self.vertices[1])]
        yield -np.sqrt(dist2[0]**2+dist2[1]**2)



    @overrides
    def is_current_done(self):
        a=(1 - 0.05 < self.vertices[0] < 1 + 0.05) and \
        (0.6 - 0.05 < self.vertices[1] < 0.6 + 0.05)
        if a: print(a)
        return a

    def calculate_pos(self):
        a = str(self.arm2.fixtures)
        match1 = re.search(r"vertices:", a).span()
        m1 = match1[1]
        ver = a[m1 + 2:]
        match2 = re.search(r"]", ver).span()
        m2 = match2[0]
        ver = ver[:m2]
        ver1 = ver.replace("(", "")
        ver2 = ver1.replace(")", "")
        ma = ver2.split(", ")

        x = (float(ma[2]) + float(ma[4])) * 0.5 + self.arm2.position[0]
        y = (float(ma[3]) + float(ma[5])) * 0.5 + self.arm2.position[1]
        return x,y





if __name__=="__main__":
    ar=ArmEnv()
    #ca.printinfo()
    #print (type(ar.extra_data.states))
    print (ar.world.bodies)
    n=1
    for state in ar.extra_data.states:
        #print (n)
        #print (find_body(ar.world, state.body))
        #print (body)
        n+=1
    #while True:
    #   ar.render()
