#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持力[N]
_GRASP_FORCE=0.2
# ボトルのtf名
_BOTTLE_TF='ar_marker/4000'
# グリッパのtf名
_HAND_TF='hand_palm_link'

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# bottleのマーカの手前0.02[m],z軸回に-1.57回転させた姿勢
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# handを0.1[m]上に移動させる姿勢
hand_up = geometry.pose(x=0.1)

# handを0.5[m]手前に移動させる姿勢
hand_back = geometry.pose(z=-0.5)

# ソファの場所
sofa_pos = (1.2, 0.4, 1.57)

if __name__=='__main__':

    # まずは一言
    # rospy.sleep(5.0)
    # tts.say('こんにちはHSRだよ。ソファ脇のペットボトルを掴もうと思います。')
    # rospy.sleep(5.0)

    try:
        print("start initialization")
        gripper.command(1.0)
        whole_body.move_to_go()
    except:
        tts.say('初期化に失敗')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # ペットボトルが見える場所に移動
        omni_base.go_abs(sofa_pos[0], sofa_pos[1], sofa_pos[2], _MOVE_TIMEOUT)
        print("finish initialization")
    except:
        tts.say('移動に失敗')
        rospy.logerr('fail to move')
        sys.exit()

    try:
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
        # 遷移後に手先を見るようにする
        whole_body.looking_hand_constraint = True

        whole_body.move_to_neutral()
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        whole_body.move_end_effector_pose([geometry.pose(z=1.0), geometry.pose(z=0.8)], ref_frame_id='hand_palm_link')
        # # ペットボトルの手前に手を持ってくる
        # whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        # # 力を指定して把持する
        # gripper.apply_force(_GRASP_FORCE)
        # # シミュレータのgrasp hackのための待ち時間。実機では不要
        # rospy.sleep(2.0)
        # # 手先相対で上にハンドを移動
        # whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # # 手先相対で後ろにハンドを移動
        # whole_body.move_end_effector_pose(hand_back, _HAND_TF)
        # # 初期姿勢に遷移
        # whole_body.move_to_neutral()
    except:
        tts.say('把持失敗')
        rospy.logerr('fail to grasp')
        sys.exit()

