from rllab.envs.base import Env
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np
#from rllab.envs.box2d.box2d_viewer import Box2DViewer


class PointEnv(Env):
    @property
    def observation_space(self):
        return Box(low=-np.inf, high=np.inf, shape=(2,))

    @property
    def action_space(self):
        return Box(low=-0.1, high=0.1, shape=(2,))

    def reset(self):
        while True:
            self._state = np.random.uniform(-1, 1, size=(2,))
            if not self.obstacle_init(self._state):
                break

        observation = np.copy(self._state)
        #print('reset')
        return observation

    def step(self, action):
        self._state = self._state + action
        self.x, self.y = self._state

        if self.obstacle(self._state):
            #self._state=self._state-action
            #self._state=self._state-action
            #self._state=self._state - action
            #self.x, self.y = self._state
            #reward=-(10)#+(self.x ** 2 + self.y ** 2) ** 0.5)
            print ('ooooops!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        '''
        else:

            self.x, self.y = self._state
            reward = - (self.x ** 2 + self.y ** 2) ** 0.5
        #print(self._state)
        #print ('reward:',reward)
        '''
        standard_deviation = 0.3
        self.z=0
        for i in range(5):

            zt=np.exp(-((self.x-self.ob[i][0])**2 + (self.y - self.ob[i][1])**2)/(2*(standard_deviation**2)))
            zt = zt / (np.sqrt(2 * np.pi) * standard_deviation)
            self.z=self.z+zt
        reward=-self.z- (self.x ** 2 + self.y ** 2) ** 0.5

        done = abs(self.x) < 0.01 and abs(self.y) < 0.01
        #if done==True:

        #   print ('done:',done)
        #if self.obstacle(self._state): print ('oooops!!!!!!!!!!!!!!!!!!!!!')
        next_observation = np.copy(self._state)
        return Step(observation=next_observation, reward=reward, done=done)

    def render(self):
        '''
        if not self.viewer:
            self.viewer = Box2DViewer(self.world)
        #if states or actions or pause:
        #    raise NotImplementedError
        if not self.viewer:
            self.start_viewer()
        if self.viewer:
            self.viewer.loop_once()
        '''
        print('current state:', self._state)

    def eudist(self,coords1, coords2):
        """ Calculates the euclidean distance between 2 lists of coordinates. """
        dist = 0
        for (x, y) in zip(coords1, coords2):
            dist += (x - y) ** 2
        return dist ** 0.5

    def obstacle(self,point):
        #Initialise 5 obstacles
        ob1=np.array([0.7,0.6])
        ob2=np.array([0.6,0.5])
        ob3=np.array([-0.8,-0.5])
        ob4=np.array([-0.6,0.4])
        ob5=np.array([0.5,-0.4])
        self.ob=[ob1,ob2,ob3,ob4,ob5]
        #To judge whether the point is near the obstacles or not(if the distance <0.1, we think it is near)
        for i in self.ob:
            if self.eudist(point,i)<0.01:
                return True

        return False

    def obstacle_init(self,point):
        #Initialise 5 obstacles
        ob1=np.array([0.7,0.6])
        ob2=np.array([0.6,0.5])
        ob3=np.array([-0.8,-0.5])
        ob4=np.array([-0.6,0.4])
        ob5=np.array([0.5,-0.4])
        self.ob=[ob1,ob2,ob3,ob4,ob5]
        #To judge whether the point is near the obstacles or not(if the distance <0.1, we think it is near)
        for i in self.ob:
            if self.eudist(point,i)<10*0.01:
                return True

        return False