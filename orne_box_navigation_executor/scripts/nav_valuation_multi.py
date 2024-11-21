#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from waypoint_manager_msgs.msg import Waypoint
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
import time

min_dist1 = 1e4
min_dist2 = 1e4
min_ttc1 = 1e4
min_ttc2 = 1e4
start_time = None
navigation_duration = 0.0
navigation_started = False
navigation_finished = False
total_distance = 0.0
prev_pos = None
prev_time = None
sum_dist1 = 0.0
sum_dist2 = 0.0
dist_count = 0

def callback_dist(data):
    global min_dist1, min_dist2, min_ttc1, min_ttc2, navigation_finished, sum_dist1, sum_dist2, dist_count, prev_time
    try:
        if navigation_finished:
            return

        # モデル名とポーズのリストを取得
        model_names = data.name
        poses = data.pose
        twists = data.twist

        # orne_boxとactor2のインデックスを取得
        orne_box_index = model_names.index('orne_box')
        actor1_index = model_names.index('actor1')
        actor2_index = model_names.index('actor2')

        # orne_boxとactor2の位置と速度を取得
        orne_box_position = calc_pose(poses, orne_box_index)
        actor1_position = calc_pose(poses, actor1_index)
        actor2_position = calc_pose(poses, actor2_index)

        orne_box_velocity = np.array([twists[orne_box_index].linear.x, twists[orne_box_index].linear.y])
        actor1_velocity = np.array([twists[actor1_index].linear.x, twists[actor1_index].linear.y])
        actor2_velocity = np.array([twists[actor2_index].linear.x, twists[actor2_index].linear.y])

        # ユークリッド距離を計算
        distance1 = np.linalg.norm(orne_box_position - actor1_position)
        distance2 = np.linalg.norm(orne_box_position - actor2_position)
        sum_dist1 += distance1
        sum_dist2 += distance2
        dist_count += 1

        min_dist1 = min(min_dist1, distance1)
        min_dist2 = min(min_dist2, distance2)

        # 相対速度を計算
        relative_velocity1 = orne_box_velocity - actor1_velocity
        relative_velocity2 = orne_box_velocity - actor2_velocity

        # TTCを計算
        if np.linalg.norm(relative_velocity1) > 1e-6:  # ゼロ除算回避
            ttc = distance1 / np.linalg.norm(relative_velocity1)
            if ttc > 0: # TTCが正の値の場合のみ更新
                min_ttc1 = min(min_ttc1, ttc)

        if np.linalg.norm(relative_velocity2) > 1e-6:  # ゼロ除算回避
            ttc = distance2 / np.linalg.norm(relative_velocity2)
            if ttc > 0: # TTCが正の値の場合のみ更新
                min_ttc2 = min(min_ttc2, ttc)

    except ValueError:
        # orne_boxまたはactor2が見つからない場合の処理
        rospy.logwarn("orne_box or actor2 not found in ModelStates message")
    
def calc_pose(poses, index_):
    return np.array([poses[index_].position.x, poses[index_].position.y])

def start_navigation(data):
    global start_time, navigation_started, total_distance, prev_pos, prev_time
    if not navigation_started:
        start_time = time.time()
        navigation_started = True
        total_distance = 0.0  # 走行距離の初期化
        prev_pos = None # 以前の位置の初期化
        prev_time = time.time()
        rospy.loginfo("Navigation started.")

def finish_navigation(data):
    global start_time, navigation_duration, navigation_finished, total_distance, sum_dist1, sum_dist2, dist_count
    if start_time is not None and data.data and not navigation_finished:
        navigation_duration = time.time() - start_time
        navigation_finished = True
        avg_speed = total_distance / navigation_duration if navigation_duration > 0 else 0.0
        avg_dist1 = sum_dist1 / dist_count if dist_count > 0 else 0.0
        avg_dist2 = sum_dist2 / dist_count if dist_count > 0 else 0.0
        rospy.loginfo("Navigation finished. Duration: %.3f seconds", navigation_duration)
        rospy.loginfo(f'TIME: {navigation_duration:.3f} sec, DISTANCE: {total_distance:.3f} m, MIN_DIST_1: {min_dist1:.3f} m, , MIN_DIST_2: {min_dist2:.3f} m, AVG_SPEED: {avg_speed:.3f} m/s, AVG_DIST_1: {avg_dist1:.3f} m, AVG_DIST_2: {avg_dist2:.3f} m, MIN_TTC_1: {min_ttc1:.3f} s, MIN_TTC_2: {min_ttc2:.3f} s')

def odom_callback(data):
    global total_distance, prev_pos, navigation_started, navigation_finished, prev_time
    if navigation_started and not navigation_finished:
        current_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        current_time = time.time()
        if prev_pos is not None:
            delta_dist = np.linalg.norm(current_pos - prev_pos)
            total_distance += delta_dist
        prev_pos = current_pos
        prev_time = current_time


def listener():
    rospy.init_node('distance_calculator', anonymous=True)
    rospy.Timer(rospy.Duration(0.1), lambda event: callback_dist(rospy.wait_for_message("/gazebo/model_states", ModelStates))) # ModelStatesの購読
    rospy.Subscriber("/odometry/filtered", Odometry, odom_callback) # Odometryの購読

    start_sub = rospy.Subscriber("/waypoint_manager/waypoint", Waypoint, start_navigation)
    finish_sub = rospy.Subscriber("/waypoint_manager/waypoint/is_reached", Bool, finish_navigation)
    
    rospy.spin()

if __name__ == '__main__':
    listener()