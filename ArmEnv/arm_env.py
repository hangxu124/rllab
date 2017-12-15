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
            self.model_path("arm.xml.mako"),
            *args, **kwargs
        )
        self.arm1 = find_body(self.world, "arm1")
        self.arm2 = find_body(self.world, "arm2")
        self.point = find_body(self.world, "point")
        Serializable.__init__(self, *args, **kwargs)


    @overides
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


    @overides
    def compute_reward(self, action):
        yield
        vertices[0] = compute_rect_vertices()
        dist1 = [(0.1 - self.arm1.vertices[0]), (-0.5 - self.arm1.vertices[1])]
        dist2 = [(0.1 - self.arm2.vertices[0]), (-0.5 - self.arm2.vertices[1])]
        yield -np.sqrt(dist2[0]**2+dist2[1]**2)



    @overrides
    def is_current_done(self):
        return (0.1 - 0.05 < self.arm2.vertices[0] < 0.1 + 0.05) and \
               (-0.5 - 0.05 < self.arm2.vertices[1] < -0.5 + 0.05)
