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
from model.CAE import CAE
from torobo_func import publish_joint_trajectory, follow_trajectory

from trajectory_msgs.msg import JointTrajectory


class ManagerBase:
    def read_argv(self):
        # if len(sys.argv) == 5:
        #     self._open_rate = float(sys.argv[1])
        #     # self._container = int(sys.argv[2])
        #     # self._cf_num = int(sys.argv[3])
        #     # self._cs_num = int(sys.argv[4])
        #     self._cf_num = int(sys.argv[2])

        #     self._container = int(sys.argv[4])
        #     self._cs_num = int(sys.argv[3])
        self._open_rate = 1
        self._cf_num = 90
        self._cs_num = 10

        if len(sys.argv) == 2:
            self._container = int(sys.argv[1])

    def init_ros(self):
        rospy.init_node("online")
        TOPIC_NAME = "/torobo/arm_controller/command"
        self._publisher = rospy.Publisher(TOPIC_NAME, JointTrajectory, queue_size=1)
        self._JOINT_NAMES = [
            "arm/joint_" + str(i) for i in range(1, 8)
        ]  # from joint_1 to joint_8

    def read_dir(self):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        DATA_DIR = CURRENT_DIR + "/../../../data/"
        self._data_dir = DATA_DIR
        self._model_dir = DATA_DIR + "model/"

    def set_MTRNN(self):
        pass
        # in_size, out_size = 30, 30
        # self._net = MTRNN(
        #     layer_size={"in": in_size, "out": out_size,
        #                 "io": 50, "cf": self._cf_num, "cs": self._cs_num},
        #     tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 30},
        #     open_rate=self._open_rate,
        #     activate=torch.nn.Tanh()
        # )
        # model_path = self._model_dir + \
        #     "MTRNN/1210/size/open01/1000/{}_{}.pth".format(
        #         self._cf_num, self._cs_num)
        # checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        # self._net.load_state_dict(checkpoint["model"])
        # print(self._net)
        # self._net.eval()
        # self._net.init_state(1)

    def add(self):
        pass

    def temp4offline(self, inputs_t):
        return self._net(inputs_t)

    def run(self):
        outputs = []
        # output_imgs = []
        cf_states = []
        cs_states = []
        hz = 5
        high_freq = 4
        rate = rospy.Rate(hz * high_freq)
        motion_count = 0
        start_step = 0  # 10 * high_freq

        print("start")
        rospy.sleep(1)
        self._dataset.start()

        while not rospy.is_shutdown() and motion_count < self.end_step * high_freq:
            motion_count += 1
            self._dataset.cal_high_freq()
            if motion_count % high_freq == 0:
                self._dataset.cal_features(high_freq)
                inputs_t = self._dataset.get_connected_data()
                if inputs_t is not None:
                    self.add()
                    cf_states.append(self._net.cf_state.detach().numpy()[0])
                    cs_states.append(self._net.cs_state.detach().numpy()[0])

                    # output = self._net(inputs_t)
                    output = self.temp4offline(inputs_t)
                    output = output.detach().numpy()[0]
                    normalized_position = output[:7]
                    joint_position = self._dataset.reverse_position(normalized_position)
                    # old_posi = self._dataset.reverse_position(inputs_t[0, :7])
                    # velocities = (joint_position - old_posi) * hz
                    # velocities = [(p-op)*10 for (p, op)
                    #               in zip(joint_position, old_posi)]
                    # print(velocities)
                    if start_step < motion_count:
                        publish_joint_trajectory(
                            self._publisher,
                            # '/torobo/arm_controller/follow_joint_trajectory',
                            self._JOINT_NAMES,
                            joint_position,
                            # velocities=velocities,
                            time_from_start=1,
                        )
                    print(motion_count / high_freq)
                    outputs.append(output)
            rate.sleep()
        add_word = self.set_addword()
        self._dataset.save_inputs(outputs, cf_states, cs_states, add_word)

    def set_addword(self):
        return "cf{}_cs{}_type{:02d}_open{:02d}_".format(
            self._cf_num, self._cs_num, self._container, int(self._open_rate * 10)
        )

    def set_cae(self, name):
        cae = CAE()
        model_path = self._model_dir + name
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        cae.load_state_dict(checkpoint["model"])
        print(cae)
        cae.eval()
        self._dataset.set_cae(cae)

    def init(self, before_scale, end_step):
        self.read_argv()
        self.read_dir()
        self.init_ros()
        self._dataset = MyDataSet()
        self._dataset.set_before_scale(before_scale)
        self.set_MTRNN()
        self.end_step = end_step
