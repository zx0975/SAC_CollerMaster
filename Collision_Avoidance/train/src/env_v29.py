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
import skfuzzy as fuzz
from skfuzzy import control as ctrl
class fuzzy:
    def __init__(self):
        
        self.F_M_output = 0
        self.Delta_D_output = 0
        self.r_f_output = 0

        F_ins = ctrl.Antecedent(np.arange(0, 100, 20), 'F_ins')
        M_ins = ctrl.Antecedent(np.arange(0, 5, 1), 'M_ins')
        F_M = ctrl.Consequent(np.arange(-1, 0.25, 0.25), 'F_M')

        Dis_z = ctrl.Antecedent(np.arange(0, 25, 5), 'Dis_z')
        Delta_z = ctrl.Antecedent(np.arange(0, 25, 5), 'Delta_z')
        Delta_D = ctrl.Consequent(np.arange(-1, 0.25, 0.25), 'Delta_D')

        F_M_S = ctrl.Antecedent(np.arange(-1, 0.25, 0.25), 'F_M_S')
        Delta_D_S = ctrl.Antecedent(np.arange(-1, 0.25, 0.25), 'Delta_D_S')
        r_f = ctrl.Consequent(np.arange(-1, 0.25, 0.25), 'r_f')

        F_ins['VB'] = fuzz.trimf(F_ins.universe, [60, 80, 100])
        F_ins['B'] = fuzz.trimf(F_ins.universe, [40, 60, 80])
        F_ins['N'] = fuzz.trimf(F_ins.universe, [20, 40, 60])
        F_ins['G'] = fuzz.trimf(F_ins.universe, [0, 20, 40])
        F_ins['VG'] = fuzz.trimf(F_ins.universe, [-20, 0, 20])

        M_ins['VB'] = fuzz.trimf(M_ins.universe, [3, 4, 5])
        M_ins['B'] = fuzz.trimf(M_ins.universe, [2, 3, 4])
        M_ins['N'] = fuzz.trimf(M_ins.universe, [1, 2, 3])
        M_ins['G'] = fuzz.trimf(M_ins.universe, [0, 1, 2])
        M_ins['VG'] = fuzz.trimf(M_ins.universe, [-1, 0, 1])

        F_M['VB'] = fuzz.trimf(F_M.universe, [-1.25, -1, -0.75])
        F_M['B'] = fuzz.trimf(F_M.universe, [-1, -0.75, -0.5])
        F_M['N'] = fuzz.trimf(F_M.universe, [-0.75, -0.5, -0.25])
        F_M['G'] = fuzz.trimf(F_M.universe, [-0.5, -0.25, 0])
        F_M['VG'] = fuzz.trimf(F_M.universe, [-0.25, 0, 0.25])

        Dis_z['VB'] = fuzz.trimf(Dis_z.universe, [15, 20, 25])
        Dis_z['B'] = fuzz.trimf(Dis_z.universe, [10, 15, 20])
        Dis_z['N'] = fuzz.trimf(Dis_z.universe, [5, 10, 15])
        Dis_z['G'] = fuzz.trimf(Dis_z.universe, [0, 5, 10])
        Dis_z['VG'] = fuzz.trimf(Dis_z.universe, [-5, 0, 5])

        Delta_z['VB'] = fuzz.trimf(Delta_z.universe, [-5, 0, 5])
        Delta_z['B'] = fuzz.trimf(Delta_z.universe, [0, 5, 10])
        Delta_z['N'] = fuzz.trimf(Delta_z.universe, [5, 10, 15])
        Delta_z['G'] = fuzz.trimf(Delta_z.universe, [10, 15, 20])
        Delta_z['VG'] = fuzz.trimf(Delta_z.universe, [15, 20, 25])

        Delta_D['VB'] = fuzz.trimf(Delta_D.universe, [-1.25, -1, -0.75])
        Delta_D['B'] = fuzz.trimf(Delta_D.universe, [-1, -0.75, -0.5])
        Delta_D['N'] = fuzz.trimf(Delta_D.universe, [-0.75, -0.5, -0.25])
        Delta_D['G'] = fuzz.trimf(Delta_D.universe, [-0.5, -0.25, 0])
        Delta_D['VG'] = fuzz.trimf(Delta_D.universe, [-0.25, 0, 0.25])

        F_M_S['VB'] = fuzz.trimf(F_M_S.universe, [-1.25, -1, -0.75])
        F_M_S['B'] = fuzz.trimf(F_M_S.universe, [-1, -0.75, -0.5])
        F_M_S['N'] = fuzz.trimf(F_M_S.universe, [-0.75, -0.5, -0.25])
        F_M_S['G'] = fuzz.trimf(F_M_S.universe, [-0.5, -0.25, 0])
        F_M_S['VG'] = fuzz.trimf(F_M_S.universe, [-0.25, 0, 0.25])

        Delta_D_S['VB'] = fuzz.trimf(Delta_D_S.universe, [-1.25, -1, -0.75])
        Delta_D_S['B'] = fuzz.trimf(Delta_D_S.universe, [-1, -0.75, -0.5])
        Delta_D_S['N'] = fuzz.trimf(Delta_D_S.universe, [-0.75, -0.5, -0.25])
        Delta_D_S['G'] = fuzz.trimf(Delta_D_S.universe, [-0.5, -0.25, 0])
        Delta_D_S['VG'] = fuzz.trimf(Delta_D_S.universe, [-0.25, 0, 0.25])

        r_f['VB'] = fuzz.trimf(r_f.universe, [-1.25, -1, -0.75])
        r_f['B'] = fuzz.trimf(r_f.universe, [-1, -0.75, -0.5])
        r_f['N'] = fuzz.trimf(r_f.universe, [-0.75, -0.5, -0.25])
        r_f['G'] = fuzz.trimf(r_f.universe, [-0.5, -0.25, 0])
        r_f['VG'] = fuzz.trimf(r_f.universe, [-0.25, 0, 0.25])

        rule1 = ctrl.Rule(F_ins['VB'] & M_ins['VB'],F_M['VB'])
        rule2 = ctrl.Rule(F_ins['VB'] & M_ins['B'],F_M['VB'])
        rule3 = ctrl.Rule(F_ins['VB'] & M_ins['N'],F_M['B'])
        rule4 = ctrl.Rule(F_ins['VB'] & M_ins['G'],F_M['N'])
        rule5 = ctrl.Rule(F_ins['VB'] & M_ins['VG'],F_M['N'])

        rule6 = ctrl.Rule(F_ins['B'] & M_ins['VB'],F_M['VB'])
        rule7 = ctrl.Rule(F_ins['B'] & M_ins['B'],F_M['VB'])
        rule8 = ctrl.Rule(F_ins['B'] & M_ins['N'],F_M['N'])
        rule9 = ctrl.Rule(F_ins['B'] & M_ins['G'],F_M['N'])
        rule10 = ctrl.Rule(F_ins['B'] & M_ins['VG'],F_M['N'])

        rule11 = ctrl.Rule(F_ins['N'] & M_ins['VB'],F_M['N'])
        rule12 = ctrl.Rule(F_ins['N'] & M_ins['B'],F_M['N'])
        rule13 = ctrl.Rule(F_ins['N'] & M_ins['N'],F_M['G'])
        rule14 = ctrl.Rule(F_ins['N'] & M_ins['G'],F_M['VG'])
        rule15 = ctrl.Rule(F_ins['N'] & M_ins['VG'],F_M['VG'])

        rule16 = ctrl.Rule(F_ins['G'] & M_ins['VB'],F_M['G'])
        rule17 = ctrl.Rule(F_ins['G'] & M_ins['B'],F_M['G'])
        rule18 = ctrl.Rule(F_ins['G'] & M_ins['N'],F_M['G'])
        rule19 = ctrl.Rule(F_ins['G'] & M_ins['G'],F_M['VG'])
        rule20 = ctrl.Rule(F_ins['G'] & M_ins['VG'],F_M['VG'])

        rule21 = ctrl.Rule(F_ins['VG'] & M_ins['VB'],F_M['G'])
        rule22 = ctrl.Rule(F_ins['VG'] & M_ins['B'],F_M['G'])
        rule23 = ctrl.Rule(F_ins['VG'] & M_ins['N'],F_M['G'])
        rule24 = ctrl.Rule(F_ins['VG'] & M_ins['G'],F_M['VG'])
        rule25 = ctrl.Rule(F_ins['VG'] & M_ins['VG'],F_M['VG'])

        rule26 = ctrl.Rule(Dis_z['VB'] & Delta_z['VB'],Delta_D['VB'])
        rule27 = ctrl.Rule(Dis_z['VB'] & Delta_z['B'],Delta_D['B'])
        rule28 = ctrl.Rule(Dis_z['VB'] & Delta_z['N'],Delta_D['N'])
        rule29 = ctrl.Rule(Dis_z['VB'] & Delta_z['G'],Delta_D['G'])
        rule30 = ctrl.Rule(Dis_z['VB'] & Delta_z['VG'],Delta_D['G'])

        rule31 = ctrl.Rule(Dis_z['B'] & Delta_z['VB'],Delta_D['VB'])
        rule32 = ctrl.Rule(Dis_z['B'] & Delta_z['B'],Delta_D['B'])
        rule33 = ctrl.Rule(Dis_z['B'] & Delta_z['N'],Delta_D['G'])
        rule34 = ctrl.Rule(Dis_z['B'] & Delta_z['G'],Delta_D['VG'])
        rule35 = ctrl.Rule(Dis_z['B'] & Delta_z['VG'],Delta_D['VG'])

        rule36 = ctrl.Rule(Dis_z['N'] & Delta_z['VB'],Delta_D['N'])
        rule37 = ctrl.Rule(Dis_z['N'] & Delta_z['B'],Delta_D['N'])
        rule38 = ctrl.Rule(Dis_z['N'] & Delta_z['N'],Delta_D['G'])
        rule39 = ctrl.Rule(Dis_z['N'] & Delta_z['G'],Delta_D['VG'])
        rule40 = ctrl.Rule(Dis_z['N'] & Delta_z['VG'],Delta_D['VG'])

        rule41 = ctrl.Rule(Dis_z['G'] & Delta_z['VB'],Delta_D['G'])
        rule42 = ctrl.Rule(Dis_z['G'] & Delta_z['B'],Delta_D['G'])
        rule43 = ctrl.Rule(Dis_z['G'] & Delta_z['N'],Delta_D['G'])
        rule44 = ctrl.Rule(Dis_z['G'] & Delta_z['G'],Delta_D['VG'])
        rule45 = ctrl.Rule(Dis_z['G'] & Delta_z['VG'],Delta_D['VG'])

        rule46 = ctrl.Rule(Dis_z['VG'] & Delta_z['VB'],Delta_D['G'])
        rule47 = ctrl.Rule(Dis_z['VG'] & Delta_z['B'],Delta_D['G'])
        rule48 = ctrl.Rule(Dis_z['VG'] & Delta_z['N'],Delta_D['G'])
        rule49 = ctrl.Rule(Dis_z['VG'] & Delta_z['G'],Delta_D['VG'])
        rule50 = ctrl.Rule(Dis_z['VG'] & Delta_z['VG'],Delta_D['VG'])

        rule51 = ctrl.Rule(F_M_S['VB'] & Delta_D_S['VB'],r_f['VB'])
        rule52 = ctrl.Rule(F_M_S['VB'] & Delta_D_S['B'],r_f['VB'])
        rule53 = ctrl.Rule(F_M_S['VB'] & Delta_D_S['N'],r_f['B'])
        rule54 = ctrl.Rule(F_M_S['VB'] & Delta_D_S['G'],r_f['N'])
        rule55 = ctrl.Rule(F_M_S['VB'] & Delta_D_S['VG'],r_f['N'])

        rule56 = ctrl.Rule(F_M_S['B'] & Delta_D_S['VB'],r_f['VB'])
        rule57 = ctrl.Rule(F_M_S['B'] & Delta_D_S['B'],r_f['N'])
        rule58 = ctrl.Rule(F_M_S['B'] & Delta_D_S['N'],r_f['N'])
        rule59 = ctrl.Rule(F_M_S['B'] & Delta_D_S['G'],r_f['G'])
        rule60 = ctrl.Rule(F_M_S['B'] & Delta_D_S['VG'],r_f['G'])

        rule61 = ctrl.Rule(F_M_S['N'] & Delta_D_S['VB'],r_f['N'])
        rule62 = ctrl.Rule(F_M_S['N'] & Delta_D_S['B'],r_f['N'])
        rule63 = ctrl.Rule(F_M_S['N'] & Delta_D_S['N'],r_f['G'])
        rule64 = ctrl.Rule(F_M_S['N'] & Delta_D_S['G'],r_f['G'])
        rule65 = ctrl.Rule(F_M_S['N'] & Delta_D_S['VG'],r_f['VG'])

        rule66 = ctrl.Rule(F_M_S['G'] & Delta_D_S['VB'],r_f['G'])
        rule67 = ctrl.Rule(F_M_S['G'] & Delta_D_S['B'],r_f['G'])
        rule68 = ctrl.Rule(F_M_S['G'] & Delta_D_S['N'],r_f['G'])
        rule69 = ctrl.Rule(F_M_S['G'] & Delta_D_S['G'],r_f['VG'])
        rule70 = ctrl.Rule(F_M_S['G'] & Delta_D_S['VG'],r_f['VG'])

        rule71 = ctrl.Rule(F_M_S['VG'] & Delta_D_S['VB'],r_f['G'])
        rule72 = ctrl.Rule(F_M_S['VG'] & Delta_D_S['B'],r_f['VG'])
        rule73 = ctrl.Rule(F_M_S['VG'] & Delta_D_S['N'],r_f['VG'])
        rule74 = ctrl.Rule(F_M_S['VG'] & Delta_D_S['G'],r_f['VG'])
        rule75 = ctrl.Rule(F_M_S['VG'] & Delta_D_S['VG'],r_f['VG'])

        F_M_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25])
        self.F_M_fuzzy = ctrl.ControlSystemSimulation(F_M_ctrl)

        Delta_D_ctrl = ctrl.ControlSystem([rule26, rule27, rule28, rule29, rule30, rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40, rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50])
        self.Delta_D_fuzzy = ctrl.ControlSystemSimulation(Delta_D_ctrl)

        r_f_ctrl = ctrl.ControlSystem([rule51, rule52, rule53, rule54, rule55, rule56, rule57, rule58, rule59, rule60, rule61, rule62, rule63, rule64, rule65, rule66, rule67, rule68, rule69, rule70, rule71, rule72, rule73, rule74, rule75])
        self.r_f_fuzzy = ctrl.ControlSystemSimulation(r_f_ctrl)

    def f_fuzzy(self,f,m):  

        self.F_M_fuzzy.input['F_ins'] = f
        self.F_M_fuzzy.input['M_ins'] = m

        self.F_M_fuzzy.compute()
        self.F_M_output = (self.F_M_fuzzy.output['F_M'])
        return np.around(self.F_M_output,3)
    
    def z_fuzzy(self,dis,delta):

        self.Delta_D_fuzzy.input['Dis_z'] = dis
        self.Delta_D_fuzzy.input['Delta_z'] = delta

        self.Delta_D_fuzzy.compute()
        self.Delta_D_output = (self.Delta_D_fuzzy.output['Delta_D'])

        return np.around(self.Delta_D_output,3)
    
    def r_f(self,f_m,delta_d):

        self.r_f_fuzzy.input['F_M_S'] = f_m
        self.r_f_fuzzy.input['Delta_D_S'] = delta_d

        self.r_f_fuzzy.compute()
        self.r_f_output = (self.r_f_fuzzy.output['r_f'])

        return np.around(self.r_f_output,3)

class Test(core.Env):
    ACTION_VEC_TRANS = 1/6000
    ACTION_ORI_TRANS = 1/2000
    ACTION_PHI_TRANS = 1/2000
    NAME = ['/right_', '/right_', '/right_']   
    def __init__(self, name, workers):
        self.__name = self.NAME[name%2]
        self.workers = 'arm'
        self.fuz = fuzzy()
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
        #             # fx,fy,fz,mx,my,mz,
        #             # vx,vy,vz,va,vb,va,vd,
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
        # self.goalpos = np.array([0.0, 55.36, -6.36, 0.0, 78.91, 56.61, -80.19, -14.50])*(np.pi/180)# [0.33, 0.0, -0.56, 0, 0, 0, 0]
        self.goalpos = np.array([0.0, 56.67, -6.87, 0.0, 80.83, 57.71, -81.27, -13.51])*(np.pi/180)# [0.33, 0.0, -0.58, 0, 0, 0, 0]
        # self.goalpos = np.array([0.0, 55.48, -6.40, 0.0, 79.09, 56.71, -80.29, -14.41])*(np.pi/180)# [0.33, 0.0, -0.59, 0, 0, 0, 0]
        rpy = np.array([0.0, 0.0, 0.0, 0.0])*(1/180)
        self.goalpos = np.append(self.goalpos, self.range_cnt)
        res = self.set_goal_client(self.goalpos, rpy, self.__name)
        goal_pos = np.array(res.state)
        if not res.success:
            return self.set_goal()
        else:
            return goal_pos[:7]

    def set_old(self):
        # self.start = np.array([0.0, 63.02, -8.58, 0.0, 88.0, 63.42, -85.22, -9.45])*(np.pi/180)# [0.33, 0.0, -0.5, 0, 0, 0, 0]
        # self.start = np.array([0.0, 60.46, -8.02, 0.0, 85.47, 61.05, -83.85, -10.96])*(np.pi/180)# [0.33, 0.0, -0.55, 0, 0, 0, 0]
        # self.start = np.array([0.0, 59.81, -7.85, 0.0, 84.75, 60.46, -83.46, -11.37])*(np.pi/180)# [0.33, 0.0, -0.555, 0, 0, 0, 0]
        # self.start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])*(np.pi/180)# [0.0, -0.243, -0.923, 0.0, 0.0, 0.0, 0]
        # self.start = np.array([0.0, 59.16, -7.67, 0.0, 84.01, 59.88, -83.05, -11.79])*(np.pi/180)# [0.33, 0.0, -0.56, 0, 0, 0, 0]
        # self.start = np.array([0.0, 58.53, -7.48, 0.0, 83.25, 59.32, -82.62, -12.21])*(np.pi/180)# [0.33, 0.0, -0.565, 0, 0, 0, 0]
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
        aaaarmmm.data = self.cmd
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
            ppppossss.data = self.dis_pos
            self.arm_force.publish(ppppossss)

            ooooriiii = Float32()
            ooooriiii.data = self.dis_ori
            self.arm_force.publish(ooooriiii)

            fffforceeee = Float32()
            fffforceeee.data = self.max_force
            self.arm_force.publish(fffforceeee)

            ttttorqueeee = Float32()
            ttttorqueeee.data = self.max_torque
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

    def get_reward(self, s, ik_success, terminal, singularity):
        reward = 0.
        self.f_reward = 0.
        self.z_reward = 0.
        self.r_f_reward = 0.

        if not ik_success:
            return -1

        
        self.f_reward = self.fuz.f_fuzzy(self.max_force, self.max_torque)
        self.z_reward = self.fuz.z_fuzzy(self.dis_z, self.delta_z)
        r_f_reward = self.fuz.r_f(self.f_reward,self.z_reward)
        reward -= (r_f_reward)

        reward -= 1.0

        reward -= (self.dis_z*5)

        if singularity:
            reward -= 1

        if terminal:
            reward = 10.0
            
        return reward

        #==================================================================================


