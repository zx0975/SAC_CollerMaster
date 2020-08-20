#!/usr/bin/env python3

import threading, queue
import time
import os
import shutil
import numpy as np
import math
import rospy
import tensorflow as tf
from test_sac_v2 import SAC
from test_env_v2 import Test
from arm_control.arm_task import ArmTask
from manipulator_h_base_module_msgs.msg import P2PPose

MAX_EPISODES = 100000
MAX_EP_STEPS =  600
MEMORY_CAPACITY = 10000
BATTH_SIZE = 256
SIDE = ['right_', 'left_']
GOAL_REWARD = 800
LOAD = False
SAVE = [False, False]

def run(nameIndx):
    global cmd, move
    
    SUCCESS_ARRAY = np.zeros([1000])
    S_RATE = 0

    env = Test(nameIndx, 0) #0 = right
    arm = ArmTask(SIDE[nameIndx]+'arm')

    agent = SAC(act_dim=env.act_dim, obs_dim=env.obs_dim, name=SIDE[nameIndx])
    reset_start = False

    for cnt in range(1000):
        done_cnt = 0

        s = env.reset(reset_start)
        reset_start = True
        goal = env.get_goal
        goal = np.append(goal, 0)
        start = (s[:8])
        time.sleep(1)
        for __ in range(MAX_EP_STEPS):
            a = agent.choose_action(s)
            s, done, ik_success, singularity, fail = env.step(a)
            done_cnt += int(done)
            if fail:
                reset_start = True
            #     with open("test_F.txt","a") as F:
            #         F.write('fail')
            #         F.write('\n')
            #     with open("test_Fx.txt","a") as Fx:
            #         Fx.write('fail')
            #         Fx.write('\n')
            #     with open("test_Fy.txt","a") as Fy:
            #         Fy.write('fail')
            #         Fy.write('\n')
            #     with open("test_Fz.txt","a") as Fz:
            #         Fz.write('fail')
            #         Fz.write('\n')
            #     with open("test_M.txt","a") as M:
            #         M.write('fail')
            #         M.write('\n')
            #     with open("test_Mx.txt","a") as My:
            #         My.write('fail')
            #         My.write('\n')
            #     with open("test_My.txt","a") as My:
            #         My.write('fail')
            #         My.write('\n')
            #     with open("test_Mz.txt","a") as Mz:
            #         Mz.write('fail')
            #         Mz.write('\n')
                break

            if done_cnt > 0:    
                SUCCESS_ARRAY[cnt%1000] = 1
                reset_start = True
                # with open("test_F.txt","a") as F:
                #     F.write('success')
                #     F.write('\n')
                # with open("test_Fx.txt","a") as Fx:
                #     Fx.write('success')
                #     Fx.write('\n')
                # with open("test_Fy.txt","a") as Fy:
                #     Fy.write('success')
                #     Fy.write('\n')
                # with open("test_Fz.txt","a") as Fz:
                #     Fz.write('success')
                #     Fz.write('\n')
                # with open("test_M.txt","a") as M:
                #     M.write('success')
                #     M.write('\n')
                # with open("test_Mx.txt","a") as My:
                #     My.write('success')
                #     My.write('\n')
                # with open("test_My.txt","a") as My:
                #     My.write('success')
                #     My.write('\n')
                # with open("test_Mz.txt","a") as Mz:
                #     Mz.write('success')
                #     Mz.write('\n')
                # break
            if __ == 599:
                reset_start = True
        arm.clear_cmd()
        S_RATE = 0
        for z in SUCCESS_ARRAY:
            S_RATE += z
        print('Ep:', cnt, 's_rate:', S_RATE, 'STEPS', __)

if __name__ == '__main__':
    rospy.init_node('aL')
    threads = []
    cmd = np.zeros([2,7])
    move = [False, False]
    COORD = tf.train.Coordinator()
    
    for i in range(1):
        t = threading.Thread(target=run, args=(i,))
        threads.append(t)
    COORD.join(threads)
    for i in range(1):
        threads[i].start()
        time.sleep(10)
    rospy.spin()