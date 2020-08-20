#!/usr/bin/env python3

"""classic Acrobot task"""
import numpy as np
from numpy import sin, cos, pi
from gym import core, spaces
from gym.utils import seeding
import rospy
import math
import time
from train.srv import get_state, move_cmd, set_goal, set_start
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from std_msgs.msg import String, Float32MultiArray, Float32
class Test(core.Env):
    ACTION_VEC_TRANS = 1/3000
    ACTION_ORI_TRANS = 1/1000
    ACTION_PHI_TRANS = 1/1000
    NAME = ['/right_', '/right_', '/right_']   
    def __init__(self, name, workers):
        self.__name = self.NAME[name%2]
        self.workers = 'arm'
        # if workers == 0:
        #     self.workers = 'arm'
        # else:
        #     self.workers = 'arm'
        #     self.workers = str(workers)

        # high = np.array([1.,1.,1.,1.,1.,1.,1.,1.,   #8
        #                  0.,0.,0.,0.,0.,0.,         #6
        #                  1.,1.,1.,1.,1.,1.,1.,      #7
        #                  0.,0.,0.,0.,0.,0.])        #6
        #                                             #27
        # low = -1*high 
        #             # ox,oy,oz,oa,ob,oc,od,of,
        #             # fx,fy,fz,mx,my,mz
        #             # vx,vy,vz,va,vb,va,vd
        #             # dz,tz,dp,do,mf,mt                  
        
        # self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32) #?
        # self.action_space = spaces.Discrete(3) #?
        self.act_dim = 8
        self.obs_dim = 27
        self.state = []
        self.action = []
        self.cmd = []
        self.goal = []
        self.goal_pos = []
        self.goal_quat = []        
        self.old = []
        self.old_pos = []
        self.dis_pos = []
        self.dis_ori = []
        self.adjust = []
        self.sensor_array = []
        self.max_force = 0.0
        self.max_torque = 0.0
        self.range_cnt = 0.0
        self.rpy_range = 0.0
        self.done = True
        self.s_cnt = 0
        self.goal_err = 0.01
        self.ori_err = 0.15
        self.arm_pos = rospy.Publisher(
            self.__name + 'arm_pos',
            Float32,
            queue_size=1,
            latch=True
            )
        self.arm_ori = rospy.Publisher(
            self.__name + 'arm_ori',
            Float32,
            queue_size=1,
            latch=True
            )
        self.arm_move = rospy.Publisher(
            self.__name + 'arm_move',
            Float32MultiArray,
            queue_size=1,
            latch=True
            )
        self.arm_force = rospy.Publisher(
            self.__name + 'arm_force',
            Float32,
            queue_size=1,
            latch=True
            )
        self.arm_torque = rospy.Publisher(
            self.__name + 'arm_torque',
            Float32,
            queue_size=1,
            latch=True
            )
        self.arm_z = rospy.Publisher(
            self.__name + 'arm_z',
            Float32,
            queue_size=1,
            latch=True
            )
        self.arm_delta = rospy.Publisher(
            self.__name + 'arm_delta',
            Float32,
            queue_size=1,
            latch=True
            )
        rospy.Subscriber('/r_force_sensor',WrenchStamped,self.force_data)
        self.reset()    
    @property
    def is_success(self):
        return self.done

    @property
    def success_cnt(self):
        return self.s_cnt
        
    def get_state_client(self, name):
        service = name+self.workers+'/get_state'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.get_state_client(name)
            
        client = rospy.ServiceProxy(
            service,
            get_state
        )
        # res = client(cmd)
        res = client.call()
        return res

    def move_cmd_client(self, cmd, name):
        service = name+self.workers+'/move_cmd'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.move_cmd_client(cmd, name)
            
        client = rospy.ServiceProxy(
            service,
            move_cmd
        )
        # res = client(cmd)
        res = client.call(cmd)
        return res

    def set_start_client(self, cmd, rpy, name):
        service = name+self.workers+'/set_start'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.set_start_client(cmd, rpy, name)
            
        client = rospy.ServiceProxy(
            service,
            set_start
        )
        # res = client(cmd)
        res = client(action=cmd, rpy=rpy)
        return res

    def set_goal_client(self, cmd, rpy, name):
        service = name+self.workers+'/set_goal'
        try:
            rospy.wait_for_service(service, timeout=1.)
        except rospy.ROSException as e:
            rospy.logwarn('wait_for_service timeout')
            self.set_goal_client(cmd, rpy, name)
            
        client = rospy.ServiceProxy(
            service,
            set_goal
        )
        # res = client(cmd)
        res = client(action=cmd, rpy=rpy)
        return res
    
    # def seed(self, seed=None):
    #     self.np_random, seed = seeding.np_random(seed)
    #     return [seed]

    def force_data(self,data):
        # rospy.loginfo(data)
        self.adjust = np.array([data.wrench.force.x,\
                                      data.wrench.force.y,\
                                      data.wrench.force.z,\
                                      data.wrench.torque.x,\
                                      data.wrench.torque.y,\
                                      data.wrench.torque.z])
        self.adjust[2] -= 11.898
        self.sensor_array = self.adjust                            
        # self.sensor_array = np.array([0,0,0,0,0,0])
        # print(self.sensor_array, len(self.sensor_array))

    def reset(self):
        self.old, self.goal_quat= self.set_old()
        self.goal = self.set_goal()
        self.state = np.append(self.old, self.sensor_array[:])
        self.state = np.append(self.state, np.subtract(self.goal[:3], self.old[:3]))
        self.state = np.append(self.state, self.goal_quat[:4])
        self.dis_x = np.linalg.norm(self.goal[0] - self.old[0])
        self.dis_y = np.linalg.norm(self.goal[1] - self.old[1])
        self.dis_z = np.linalg.norm(self.goal[2] - self.old[2])
        self.delta_z = np.linalg.norm(self.state[2] + 0.55)
        self.dis_pos = np.linalg.norm(self.goal[:3] - self.old[:3])
        self.dis_ori = math.sqrt(np.linalg.norm(self.goal[3:7] - self.old[3:7]) + np.linalg.norm(-1*self.goal[3:7] - self.old[3:7]) - 2)
        self.max_force = np.linalg.norm(self.state[8:11])
        self.max_torque = np.linalg.norm(self.state[11:14]) 
        self.state = np.append(self.state, self.dis_z)
        self.state = np.append(self.state, self.delta_z)
        self.state = np.append(self.state, self.dis_pos)
        self.state = np.append(self.state, self.dis_ori)
        self.state = np.append(self.state, self.max_force)
        self.state = np.append(self.state, self.max_torque)
        self.done = False
        self.success = False

        return self.state

    def set_goal(self):
        # self.goalpos = np.array([0.0, 55.36, -6.36, 0.0, 78.91, 56.61, -80.19, -14.50])*(np.pi/180)# [0.33, 0.0, -0.56, 0, 0, 0, 1]
        self.goalpos = np.array([0.0, 56.67, -6.87, 0.0, 80.83, 57.71, -81.27, -13.51])*(np.pi/180)# [0.33, 0.0, -0.58, 0, 0, 0, 1]
        # self.goalpos = np.array([0.0, 55.48, -6.40, 0.0, 79.09, 56.71, -80.29, -14.41])*(np.pi/180)# [0.33, 0.0, -0.59, 0, 0, 0, 1]
        rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        self.goalpos = np.append(self.goalpos, self.range_cnt)
        res = self.set_goal_client(self.goalpos, rpy, self.__name)
        goal_pos = np.array(res.state)
        if not res.success:
            return self.set_goal()
        else:
            return goal_pos[:7]

    def set_old(self):
        # self.start = np.array([0.0, 63.02, -8.58, 0.0, 88.0, 63.42, -85.22, -9.45])*(np.pi/180)# [0.33, 0.0, -0.5, 0, 0, 0, 1]
        # self.start = np.array([0.0, 60.46, -8.02, 0.0, 85.47, 61.05, -83.85, -10.96])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, 0, 1]
        # self.start = np.array([0.0, 59.81, -7.85, 0.0, 84.75, 60.46, -83.46, -11.37])*(np.pi/180)# [0.33, 0.0, -0.555, 0, 0, 0, 1]
        # self.start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])*(np.pi/180)# [0.0, -0.243, -0.923, 0.0, 0.0, 0.0, 1]
        # self.start = np.array([0.0, 59.16, -7.67, 0.0, 84.01, 59.88, -83.05, -11.79])*(np.pi/180)# [0.33, 0.0, -0.56, 0, 0, 0, 1]
        # self.start = np.array([0.0, 58.53, -7.48, 0.0, 83.25, 59.32, -82.62, -12.21])*(np.pi/180)# [0.33, 0.0, -0.565, 0, 0, 0, 1]
        # rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        pos_count = np.random.randint(low=0, high=8, size=1)
        if pos_count == 0:
            self.start = np.array([0.0, 60.46, -8.02, 0.0, 85.47, 61.05, -83.85, -10.96])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, 0, 0]
            rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        elif pos_count == 1:
            self.start = np.array([0.0, 59.74, -19.74, 0.0, 94.72, 62.95, -72.76, -13.56])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, 10, 0]
            rpy = np.array([0.0, 0.0, 10.0, 0.0])*(1/180)
        elif pos_count == 2:
            self.start = np.array([0.0, 59.74, 4.99, 0.0, 70.71, 58.38, -92.62, -12.33])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, -10, 0]
            rpy = np.array([0.0, 0.0, -10.0, 0.0])*(1/180)
        elif pos_count == 3:
            self.start = np.array([0.0, 59.70, -6.61, 0.0, 86.02, 60.13, -84.68, -9.17])*(np.pi/180)# [0.32, 0.01, -0.55, 0, 0, 0, 0]
            rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        elif pos_count == 4:
            self.start = np.array([0.0, 58.97, -18.47, 0.0, 95.72, 61.57, -73.65, -11.37])*(np.pi/180)# [0.32, 0.01, -0.55, 0, 0, 10, 0]  
            rpy = np.array([0.0, 0.0, 10.0, 0.0])*(1/180)
        elif pos_count == 5:
            self.start = np.array([0.0, 58.97, 6.48, 0.0, 70.80, 57.70, -93.31, -10.89])*(np.pi/180)# [0.32, 0.01, -0.55, 0, 0, -10, 0]
            rpy = np.array([0.0, 0.0, -10.0, 0.0])*(1/180)
        elif pos_count == 6: 
            self.start = np.array([0.0, 61.19, -9.34, 0.0, 84.76, 61.97, -83.03, -12.84])*(np.pi/180)# [0.34, -0.01, -0.55, 0, 0, 0, 0]
            rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        elif pos_count == 7: 
            self.start = np.array([0.0, 60.48, -20.92, 0.0, 93.57, 64.36, -71.92, -15.84])*(np.pi/180)# [0.34, -0.01, -0.55, 0, 0, 10, 0]
            rpy = np.array([0.0, 0.0, 10.0, 0.0])*(1/180)
        elif pos_count == 8: 
            self.start = np.array([0.0, 60.48, 3.57, 0.0, 70.45, 59.03, -91.91, -13.87])*(np.pi/180)# [0.34, -0.01, -0.55, 0, 0, -10, 0]
            rpy = np.array([0.0, 0.0, -10.0, 0.0])*(1/180)
        else:
            self.start = np.array([0.0, 60.46, -8.02, 0.0, 85.47, 61.05, -83.85, -10.96])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, 0, 0]
            rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        self.start = np.append(self.start, self.range_cnt)
        res = self.set_start_client(self.start, rpy, self.__name)
        old_pos = np.array(res.state)      
        if not res.success:
            return self.set_old()
        else:
            return old_pos, res.quaterniond

    def step(self, a):      
        s = self.state
        action_vec = a[:3]*self.ACTION_VEC_TRANS
        action_ori = a[3:7]*self.ACTION_ORI_TRANS
        action_phi = a[7]*self.ACTION_PHI_TRANS
        self.action = np.append(action_vec, action_ori)
        self.action = np.append(self.action, action_phi)
        self.cmd = np.add(s[:8], self.action)
        self.cmd[3:7] /= np.linalg.norm(self.cmd[3:7])
        res = self.move_cmd_client(self.cmd, self.__name)
        aaaarmmm = Float32MultiArray()
        aaaarmmm.data =  self.cmd
        self.arm_move.publish(aaaarmmm)

        if res.success:
            self.old = np.array(res.state)
            self.goal_quat = res.quaterniond
            s = np.append(self.old, self.sensor_array[:])
            s = np.append(s, np.subtract(self.goal[:3], self.old[:3]))
            s = np.append(s, self.goal_quat[:4])
            self.dis_x = np.linalg.norm(self.goal[0] - s[0])
            self.dis_y = np.linalg.norm(self.goal[1] - s[1])
            self.dis_z = np.linalg.norm(self.goal[2] - s[2])
            self.delta_z = np.linalg.norm(s[2] + 0.55)
            self.dis_pos = np.linalg.norm(self.goal[:3] - s[:3])
            self.dis_ori = math.sqrt(np.linalg.norm(self.goal[3:7] - s[3:7]) + np.linalg.norm(-1*self.goal[3:7] - s[3:7]) - 2)
            self.max_force = np.linalg.norm(s[8:11])
            self.max_torque = np.linalg.norm(s[11:14])
            x_random = np.random.normal(loc=0.0, scale=0.001, size=1)
            y_random = np.random.normal(loc=0.0, scale=0.001, size=1)
            s[14] += x_random
            s[15] += y_random
        
            ppppossss = Float32()
            ppppossss.data =  self.dis_pos
            self.arm_pos.publish(ppppossss)

            ooooriiii = Float32()
            ooooriiii.data =  self.dis_ori
            self.arm_ori.publish(ooooriiii)

            fffforceeee = Float32()
            fffforceeee.data =  self.max_force
            self.arm_force.publish(fffforceeee)

            ttttorqueeee = Float32()
            ttttorqueeee.data =  self.max_torque
            self.arm_torque.publish(ttttorqueeee)

            zzzz = Float32()
            zzzz.data =  self.dis_z
            self.arm_z.publish(zzzz)

            ddddeltaaaa = Float32()
            ddddeltaaaa.data =  self.delta_z
            self.arm_delta.publish(ddddeltaaaa)

            s = np.append(s, self.dis_z)
            s = np.append(s, self.delta_z)
            s = np.append(s, self.dis_pos)
            s = np.append(s, self.dis_ori)
            s = np.append(s, self.max_force)
            s = np.append(s, self.max_torque)
        terminal = self._terminal(s, res.success)
        reward = self.get_reward(s, res.success, terminal, res.singularity)

        self.state = s
        fail = False
        if self.max_force > 300 or self.max_torque > 30 or self.dis_x  > 0.05 or self.dis_y > 0.05 or self.dis_pos > 0.08:
            fail = True
        if not res.success or res.singularity:
            fail = True
      
        return self.state, reward, terminal, self.success, fail

    def _terminal(self, s, ik_success):
        if ik_success:
            if self.dis_pos < self.goal_err and self.dis_ori < self.ori_err:
                self.success = True
                if not self.done:
                    self.done = True
                    self.s_cnt += 1
                    self.goal_err = self.goal_err*0.996 if self.goal_err > 0.002 else 0.002
                    self.ori_err = self.ori_err*0.996 if self.ori_err > 0.1 else 0.1
                return True
            else:
                self.success = False
                return False
        else:
            self.success = False
            return False

    # def get_reward(self, s, terminal, singularity, d_z, delta_z, f_max, m_max):
    def get_reward(self, s, ik_success, terminal, singularity):
        reward = 0.
        f_reward = 0.
        z_reward = 0.
 
        if not ik_success:
            return -1
        
        # reward -= self.dis_z
        # reward -= self.delta_z
        # print(self.max_force)
        # print(self.max_torque)
        # print(self.dis_z)
        # print(self.delta_z)
        
        if self.max_force > 80:
            if self.max_torque > 4:
                f_reward = -1.0
            elif self.max_torque > 3:
                f_reward = -1.0
            elif self.max_torque > 2:
                f_reward = -0.8
            elif self.max_torque > 1:
                f_reward = -0.6
            elif self.max_torque > 0:
                f_reward = -0.6
        elif self.max_force > 60:
            if self.max_torque > 4:
                f_reward = -1.0
            elif self.max_torque > 3:
                f_reward = -1.0
            elif self.max_torque > 2:
                f_reward = -0.6
            elif self.max_torque > 1:
                f_reward = -0.6
            elif self.max_torque > 0:
                f_reward = -0.6
        elif self.max_force > 40:
            if self.max_torque > 4:
                f_reward = -0.6
            elif self.max_torque > 3:
                f_reward = -0.6
            elif self.max_torque > 2:
                f_reward = -0.4
            elif self.max_torque > 1:
                f_reward = -0.2
            elif self.max_torque > 0:
                f_reward = -0.2
        elif self.max_force > 20:
            if self.max_torque > 4:
                f_reward = -0.4
            elif self.max_torque > 3:
                f_reward = -0.4
            elif self.max_torque > 2:
                f_reward = -0.4
            elif self.max_torque > 1:
                f_reward = -0.2
            elif self.max_torque > 0:
                f_reward = -0.2
        elif self.max_force > 0:
            if self.max_torque > 4:
                f_reward = -0.4
            elif self.max_torque > 3:
                f_reward = -0.4
            elif self.max_torque > 2:
                f_reward = -0.4
            elif self.max_torque > 1:
                f_reward = -0.2
            elif self.max_torque > 0:
                f_reward = -0.2

        if self.dis_z > 0.02:
            if self.delta_z < 0.001:
                z_reward = -1.0
            elif self.delta_z < 0.002:
                z_reward = -0.8
            elif self.delta_z < 0.003:
                z_reward = -0.6
            elif self.delta_z < 0.004:
                z_reward = -0.4
            elif self.delta_z < 0.005:
                z_reward = -0.4
        elif self.dis_z > 0.015:
            if self.delta_z < 0.001:
                z_reward = -1.0
            elif self.delta_z < 0.002:
                z_reward = -0.8
            elif self.delta_z < 0.003:
                z_reward = -0.4
            elif self.delta_z < 0.004:
                z_reward = -0.2
            elif self.delta_z < 0.005:
                z_reward = -0.2
        elif self.dis_z > 0.010:
            if self.delta_z < 0.001:
                z_reward = -0.6
            elif self.delta_z < 0.002:
                z_reward = -0.6
            elif self.delta_z < 0.003:
                z_reward = -0.4
            elif self.delta_z < 0.004:
                z_reward = -0.2
            elif self.delta_z < 0.005:
                z_reward = -0.2
        elif self.dis_z > 0.005:
            if self.delta_z < 0.001:
                z_reward = -0.4
            elif self.delta_z < 0.002:
                z_reward = -0.4
            elif self.delta_z < 0.003:
                z_reward = -0.4
            elif self.delta_z < 0.004:
                z_reward = -0.2
            elif self.delta_z < 0.005:
                z_reward = -0.2
        elif self.dis_z > 0.0:
            if self.delta_z < 0.001:
                z_reward = -0.4
            elif self.delta_z < 0.002:
                z_reward = -0.4
            elif self.delta_z < 0.003:
                z_reward = -0.4
            elif self.delta_z < 0.004:
                z_reward = -0.2
            elif self.delta_z < 0.005:
                z_reward = -0.2

        if f_reward == -1.0:
            if z_reward == -1.0:
                reward -= 1.0
            elif z_reward == -0.8:
                reward -= 1.0
            elif z_reward == -0.6:
                reward -= 0.8
            elif z_reward == -0.4:
                reward -= 0.6
            elif z_reward == -0.2:
                reward -= 0.6
        elif f_reward == -0.8:
            if z_reward == -1.0:
                reward -= 1.0
            elif z_reward == -0.8:
                reward -= 0.6
            elif z_reward == -0.6:
                reward -= 0.6
            elif z_reward == -0.4:
                reward -= 0.4
            elif z_reward == -0.2:
                reward -= 0.4
        elif f_reward == -0.6:
            if z_reward == -1.0:
                reward -= 0.6
            elif z_reward == -0.8:
                reward -= 0.6
            elif z_reward == -0.6:
                reward -= 0.4
            elif z_reward == -0.4:
                reward -= 0.4
            elif z_reward == -0.2:
                reward -= 0.2
        elif f_reward == -0.4:
            if z_reward == -1.0:
                reward -= 0.4
            elif z_reward == -0.8:
                reward -= 0.4
            elif z_reward == -0.6:
                reward -= 0.4
            elif z_reward == -0.4:
                reward -= 0.2
            elif z_reward == -0.2:
                reward -= 0.2
        elif f_reward == -0.2:
            if z_reward == -1.0:
                reward -= 0.4
            elif z_reward == -0.8:
                reward -= 0.2
            elif z_reward == -0.6:
                reward -= 0.2
            elif z_reward == -0.4:
                reward -= 0.2
            elif z_reward == -0.2:
                reward -= 0.2
        else:
            reward = -1.0
        
        reward -= 1

        # reward *= 10
        # reward += (f_reward*5)
        # reward += (z_reward*10)
        # reward -= self.delta_z
        reward -= (self.dis_z*30)
        # reward -= (self.dis_z*1000)

        if singularity:
            reward -= 1.0

        if terminal:
            reward = 10.0

        return reward

        #==================================================================================


