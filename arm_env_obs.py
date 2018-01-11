import numpy as np
from rllab.envs.box2d.parser import find_body
from rllab.core.serializable import Serializable
from rllab.envs.box2d.box2d_env import Box2DEnv
from rllab.misc import autoargs
from rllab.misc.overrides import overrides
import time
import math
#from rllab.misc.mako_utils import compute_rect_vertices


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
            self.model_path("arm_obs_env.xml.mako"),
            *args, **kwargs
        )
        self.arm1 = find_body(self.world, "arm1")
        self.arm2 = find_body(self.world, "arm2")
        self.point = find_body(self.world, "point")
        self.ob1 = find_body(self.world, "ob1")
        self.ob2 = find_body(self.world, "ob2")
        self.vertices=np.array([0.0,0.0])
        Serializable.__init__(self, *args, **kwargs)


    @overrides
    def reset(self):
        self._set_state(self.initial_state)
        self._invalidate_state_caches()
        lower, higher = -np.pi,np.pi
        aarm1 = np.random.uniform(lower,higher)
        aarm2 = np.random.uniform(lower,higher)
        self.arm1.angle = aarm1
        self.arm2.angle = aarm2
        #print(self.arm1.angle,self.arm2.angle)
        return self.get_current_obs()

    @overrides
    def compute_reward(self, action):
        yield
        self.vertices[0] ,self.vertices[1]=self.get_tip_pos()
        #dist1 = [(0.1 - self.arm1.vertices[0]), (-0.5 - self.arm1.vertices[1])]
        dist2 = [(1 - self.vertices[0]), (0.6 - self.vertices[1])]
        r1=-np.sqrt(dist2[0]**2+dist2[1]**2)
        if self.judge_obstacale(self.ob1) or self.judge_obstacale(self.ob2):
            r1-=100
            print('oh!God')
        yield r1



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

    def judge_obstacale(self,ob):
        #obstacale
        x3 = ob.position[0]
        y3 = ob.position[1]
        #arm2判定
        arm2_coord = self.compute_rect_vertices(self.arm2.position, self.get_tip_pos(), 0.05)
        #line1 手臂2左直线根据a,b两点构造直线方程 AX+BY+C=0

        A1=arm2_coord[2][1]-arm2_coord[3][1] #y2-y1
        B1=-(arm2_coord[2][0]-arm2_coord[3][0]) #x1-x2
        C1=arm2_coord[2][0]*arm2_coord[3][1]-arm2_coord[3][0]*arm2_coord[2][1]  #x2*y1-x1*y2
        #计算obstacale点到直线距离
        D1=abs(A1*x3+B1*y3+C1)/math.sqrt(A1*A1+B1*B1)
        #line2 手臂2右直线 根据a,b两点构造直线方程 AX+BY+C=0
        A2=arm2_coord[1][1]-arm2_coord[0][1]
        B2=-(arm2_coord[1][0]-arm2_coord[0][0])
        C2=arm2_coord[1][0]*arm2_coord[0][1]-arm2_coord[0][0]*arm2_coord[1][1]
        #计算obstacale点到直线距离
        D2=abs(A2*x3+B2*y3+C2)/math.sqrt(A2*A2+B2*B2)
        #line3 手臂2上直线 根据a,b两点构造直线方程 AX+BY+C=0
        A3=arm2_coord[1][1]-arm2_coord[2][1]
        B3=-(arm2_coord[1][0]-arm2_coord[2][0])
        C3=arm2_coord[1][0]*arm2_coord[2][1]-arm2_coord[2][0]*arm2_coord[1][1]
        #计算obstacale点到直线距离
        D3=abs(A3*x3+B3*y3+C3)/math.sqrt(A3*A3+B3*B3)

        A4=arm2_coord[0][1]-arm2_coord[3][1]
        B4=-(arm2_coord[0][0]-arm2_coord[3][0])
        C4=arm2_coord[0][0]*arm2_coord[3][1]-arm2_coord[3][0]*arm2_coord[0][1]

        D4=abs(A4*x3+B4*y3+C4)/math.sqrt(A4*A4+B4*B4)
        #arm1判定
        arm1_coord = self.compute_rect_vertices(self.arm1.position, self.get_tip_pos(), 0.05)

        A5=arm1_coord[2][1]-arm1_coord[3][1] #y2-y1
        B5=-(arm1_coord[2][0]-arm1_coord[3][0]) #x1-x2
        C5=arm1_coord[2][0]*arm1_coord[3][1]-arm1_coord[3][0]*arm1_coord[2][1]  #x2*y1-x1*y2

        D5=abs(A5*x3+B5*y3+C5)/math.sqrt(A5*A5+B5*B5)

        A6=arm1_coord[1][1]-arm1_coord[0][1]
        B6=-(arm1_coord[1][0]-arm1_coord[0][0])
        C6=arm1_coord[1][0]*arm1_coord[0][1]-arm1_coord[0][0]*arm1_coord[1][1]

        D6=abs(A6*x3+B6*y3+C6)/math.sqrt(A6*A6+B6*B6)

        A7=arm1_coord[1][1]-arm1_coord[2][1]
        B7=-(arm1_coord[1][0]-arm1_coord[2][0])
        C7=arm1_coord[1][0]*arm1_coord[2][1]-arm1_coord[2][0]*arm1_coord[1][1]

        D7=abs(A7*x3+B7*y3+C7)/math.sqrt(A7*A7+B7*B7)

        A8=arm1_coord[0][1]-arm1_coord[3][1]
        B8=-(arm1_coord[0][0]-arm1_coord[3][0])
        C8=arm1_coord[0][0]*arm1_coord[3][1]-arm1_coord[3][0]*arm1_coord[0][1]

        D8=abs(A8*x3+B8*y3+C8)/math.sqrt(A8*A8+B8*B8)

        #return D1<0.2 and D2<0.2 and D5<0.2 and D6<0.2 and D3<1.1 and D4<1.1 and D7<1.1 and D8<1.1
        return (D1<0.22 and D2<0.22 and D3<1.12 and D4<1.12) or (D5<0.22 and D6<0.22 and D7<1.12 and D8<1.12)

    def compute_rect_vertices(self, fromp, to, radius):
        x1, y1 = fromp
        x2, y2 = to
        if abs(y1 - y2) < 1e-6:
            dx = 0
            dy = radius
        else:
            dx = radius * 1.0 / (((x1 - x2) / (y1 - y2)) ** 2 + 1) ** 0.5
            # equivalently dx = radius * (y2-y1).to_f / ((x2-x1)**2 + (y2-y1)**2)**0.5
            dy = (radius ** 2 - dx ** 2) ** 0.5
            dy *= -1 if (x1 - x2) * (y1 - y2) > 0 else 1

        return np.array([
            [x1 + dx, y1 + dy],
            [x2 + dx, y2 + dy],
            [x2 - dx, y2 - dy],
            [x1 - dx, y1 - dy],
        ])


if __name__=="__main__":
    ar=ArmEnv()
    print (ar.ob2)
    #ca.printinfo()
    #print (type(ar.extra_data.states))
    a=[0,0]
    b=[0,0]
    ar.render()
    #time.sleep(2)
    while True:

        #print ("vertices:",ar.vertices)
        ar.step(np.array([1]))
        #print("arm2:", ar.arm2.position)
        #print ("tip:",ar.get_tip_pos())
        ar.render()
        a[0],a[1]=ar.get_tip_pos()
        b[0],b[1]=ar.arm2.position
        #print (ar.judge_obstacale())

        #print (((abs(a[0]-b[0]))**2+ (abs(a[1]-b[1])**2))**0.5)
        print (ar.judge_obstacale())
        time.sleep(0.2)