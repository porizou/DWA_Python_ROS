#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy


from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D,PoseArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

import math
from enum import Enum
import numpy as np


import time

import tf2_ros
from tf.transformations import euler_from_quaternion

class RobotType(Enum):
    circle = 0
    rectangle = 1
    
class DWA():
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("follow_robot/diff_drive_controller/odom", Odometry, self.odomCallback)
        self.goal_subscriber = rospy.Subscriber("goal", Pose2D, self.goalCallback)
        self.object_subscriber = rospy.Subscriber("object_points", Float32MultiArray,  self.objectCallback)

        # state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # goal position [x(m), y(m)]
        self.goal = np.array([0.0, 0.0])

        # obstacles [x(m) y(m), ....]
        self.object_point = np.array([[50.0, 50.0]])

        self.zero_object_point = np.array([[50.0, 50.0]])

        self.pub = rospy.Publisher("follow_robot/diff_drive_controller/cmd_vel", Twist, queue_size=10)
    
    #クオータニオンをオイラー角に変換してyaw[rad]を取得
    def getYaw(self, quat):
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        return yaw


    #現在のロボットの状態を取得
    def odomCallback(self, data):

        odom = data

        self.x[0] = odom.pose.pose.position.x
        self.x[1] = odom.pose.pose.position.y 
        
        self.x[2] = self.getYaw(odom.pose.pose.orientation)

        self.x[3] = odom.twist.twist.linear.x
        self.x[4] = odom.twist.twist.angular.z

        #print(self.x)

    #ゴール座標を取得
    def goalCallback(self, data):
        self.goal[0] = data.x
        self.goal[1] = data.y
        #print(self.goal)
        
    #障害物座標の配列を取得
    def objectCallback(self, data):
        data = np.array(data.data)

        if data.size == 0:
            self.object_point = self.zero_object_point
        else:
            self.object_point = data.reshape(data.shape[0]/2, 2)


    #制御量を出力
    def publishTwist(self, u):
        twist = Twist()
        twist.linear.x = u[0]
        twist.angular.z = u[1]

        self.pub.publish(twist)

    def dwa_control(self, x, config, goal, ob):
        """
        Dynamic Window Approach control
        """

        dw = self.calc_dynamic_window(x, config)

        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob)

        return u, trajectory


    def calc_dynamic_window(self, x, config):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
            -config.max_yawrate, config.max_yawrate]

        # Dynamic window from motion model
        Vd = [x[3] - config.max_accel * config.dt,
            x[3] + config.max_accel * config.dt,
            x[4] - config.max_dyawrate * config.dt,
            x[4] + config.max_dyawrate * config.dt]

        #  [vmin, vmax, yaw_rate min, yaw_rate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def calc_control_and_trajectory(self, x, dw, config, goal, ob):
        """
        calculation final input with dynamic window
        """
        t2 = time.time()

        x_init = x[:]
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        trajectorys = []
        v_ = np.array([])
        omega = np.array([])
        to_goal_costs = np.array([])
        speed_costs = np.array([])
        ob_costs = np.array([])

        for v in np.arange(dw[0], dw[1], config.v_reso):
            for y in np.arange(dw[2], dw[3], config.yawrate_reso):
                t5 = time.time()
                #path計算
                path = predict_trajectory(x_init, v, y, config)
                t5_ = time.time() - t5
                print(t5_ * 100 , "t5")
                v_ = np.append(v_, v)
                omega = np.append(omega, y)

                t4 = time.time()
                #コスト計算
                goal_cost = self.calc_to_goal_cost(path, goal)
                speed_cost = path[-1, 3]
                ob_cost = self.calc_obstacle_cost(path, ob, config)
                print(100 * (time.time() - t4), "t4")


                to_goal_costs = np.append(to_goal_costs, goal_cost)
                speed_costs = np.append(speed_costs, speed_cost)
                ob_costs = np.append(ob_costs, ob_cost)

        t1 = time.time()

        # 正規化
        for scores in [to_goal_costs, speed_costs, ob_costs]:
            scores = self.min_max_normalize(scores)

        #コストが最大になるPathを探索
        max_cost = 0.0
        for i in range(len(v_)):
            final_cost = 0
            final_cost = config.to_goal_cost_gain * to_goal_costs[i] + config.speed_cost_gain * speed_costs[i] + config.obstacle_cost_gain * ob_costs[i]

            if final_cost > max_cost:
                max_cost = final_cost
                best_u = [v_[i],omega[i]]
        

        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory, ob, config):
        """
            calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if (r <= config.robot_radius).any():
            return 0

        min_r = np.min(r)
        return min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return math.pi - cost

    # 正規化
    def min_max_normalize(self, data):

        max_data = np.max(data)
        min_data = np.min(data)

        if max_data - min_data == 0:
            data = [0.0 for i in range(len(data))]
        else:
            data = (data - min_data) / (max_data - min_data)

        return data



def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """
    x = np.array(x_init)
    traj = np.array(x)
    t = 0
    while t <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        
        t1 = time.time()
        traj = np.vstack((traj, x))
        print(time.time() - t1)
        t += config.dt

    return traj