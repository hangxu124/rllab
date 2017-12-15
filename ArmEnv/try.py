import numpy as np
from rllab.envs.box2d.parser import find_body
import re
from rllab.core.serializable import Serializable
from rllab.envs.box2d.box2d_env import Box2DEnv
from rllab.misc import autoargs
from rllab.misc.overrides import overrides


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



if __name__=="__main__":
    ca=ArmEnv()
    ca.printinfo()

    while True:
       ca.render()
