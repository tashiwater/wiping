#!/usr/bin/env python
# coding: utf-8

import os
import torch
import torchvision
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

# from sklearn.decomposition import PCA
import rospy

from dataset.dataset_base import OnlineDataSet as MyDataSet
from model.MTRNN_cs import MTRNN
from torobo_func import follow_trajectory
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class ManagerBase:
    def read_argv(self):
        if len(sys.argv) == 5:
            self._open_rate = float(sys.argv[1])
            # self._container = int(sys.argv[2])
            # self._cf_num = int(sys.argv[3])
            # self._cs_num = int(sys.argv[4])
            self._cf_num = int(sys.argv[2])

            self._container = int(sys.argv[3])
            self._cs_num = int(sys.argv[4])

    def init_ros(self):
        rospy.init_node("online")
        self._action_service_name = "/torobo/arm_controller/follow_joint_trajectory"
        self._JOINT_NAMES = ["arm/joint_" + str(i)
                             for i in range(1, 8)]  # from joint_1 to joint_8

    def read_dir(self):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        DATA_DIR = CURRENT_DIR + "/../../../data/"
        self._result_dir = DATA_DIR + "result/"
        self._model_dir = DATA_DIR + "model/"

    def set_MTRNN(self):
        in_size, out_size = 30, 30
        self._net = MTRNN(
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs": self._cs_num},
            tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/1210/size/open01/1000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        self._net.init_state(1)

    def run(self):
        outputs = []
        # output_imgs = []
        cf_states = []
        cs_states = []
        hz = 10
        high_freq = 4
        rate = rospy.Rate(hz * high_freq)
        end_step = 20 * hz
        motion_count = 0
        start_frame = 0  # 10 * high_freq

        print("start")
        for _ in range(5):
            rate.sleep()
        self._dataset.start()

        while not rospy.is_shutdown() and motion_count < end_step:
            self._dataset.cal_high_freq()
            self._dataset.cal_features(4)
            inputs_t = self._dataset.get_connected_data()
            if inputs_t is not None:
                motion_count += 1

                cf_states.append(self._net.cf_state.detach().numpy()[0])
                cs_states.append(self._net.cs_state.detach().numpy()[0])
                output = self._net(inputs_t)
                output = output.detach().numpy()[0]
                normalized_position = output[:7]
                joint_position = self._dataset.reverse_position(
                    normalized_position)
                if motion_count > start_frame:
                    ret = follow_trajectory(
                        self._action_service_name,
                        joint_names=self._JOINT_NAMES,
                        positions=joint_position,
                        time_from_start=1,
                    )
                    print(ret)
                    print(motion_count)
                outputs.append(output)
            rate.sleep()
        add_word = "cf{}_cs{}_type{:02d}_open{:02d}_".format(
            self._cf_num, self._cs_num, self._container, int(self._open_rate*10))
        self._dataset.save_inputs(outputs, cf_states, cs_states, add_word)

    def init(self, before_scale):
        self.read_argv()
        self.read_dir()
        self.init_ros()
        self._dataset = MyDataSet()
        self._dataset.set_before_scale(before_scale)
        self.set_MTRNN()
