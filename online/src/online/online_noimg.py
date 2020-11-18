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

from dataset_noimg import OnlineDataSet as MyDataSet
from model.MTRNN import MTRNN
from model.CNNMTRNN import CNNMTRNN
from model.CAE import CAE as CAE
from torobo_func import follow_trajectory
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

if len(sys.argv) != 2:
    container = int(sys.argv[2])
    open_rate = float(sys.argv[1])


rospy.init_node("online")
action_service_name = "/torobo/arm_controller/follow_joint_trajectory"
JOINT_NAMES = ["arm/joint_" + str(i)
               for i in range(1, 8)]  # from joint_1 to joint_8

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = CURRENT_DIR + "/../../data/"
RESULT_DIR = DATA_DIR + "result/"
MODEL_DIR = DATA_DIR + "model/"

cae = None
# cae = CAE()
# cae_path = MODEL_DIR + "CAE/20201116_174430_2000.pth"
# checkpoint = torch.load(cae_path, map_location=torch.device("cpu"))
# cae.load_state_dict(checkpoint["model"])
high_freq = 4
mode = "normal"
device = torch.device("cuda:0")
dataset = MyDataSet(cae, device, high_freq, mode)

in_size = 30
cf_num = 90
cs_num = 10
net = MTRNN(
    layer_size={"in": in_size, "out": in_size,
                "io": 50, "cf": cf_num, "cs": cs_num},
    tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 30},
    open_rate=open_rate,
    activate=torch.nn.Tanh()
)
model_path = MODEL_DIR + "MTRNN/1116_10000/{}_{}.pth".format(cf_num, cs_num)
checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
net.load_state_dict(checkpoint["model"])
# net = CNNMTRNN(
#     layer_size={"in": in_size, "out": in_size, "io": 50, "cf": 100, "cs": 15},
#     tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 50},
#     open_rate=0.9
# )
# model_path = MODEL_DIR + "CNNMTRNN/20201107_085638_4000.pth"
# checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
# net.load_state_dict(checkpoint["model"])
# net.to(device)

print(net)
net.eval()
net.init_state(1)
alltype_cs = []
outputs = []
output_imgs = []
io_states = []
cf_states = []
cs_states = []
hz = 10
rate = rospy.Rate(hz * high_freq)
end_step = 30 * hz
motion_count = 0
start_frame = 0  # 10 * high_freq

finish_s = 50
start_s = rospy.Time.now().to_sec()
now_s = 0
old_s = 0
output_interval_s = 1.0 / hz - 0.01

print("start")
for i in range(5):
    dataset.cal_high_freq()
    rate.sleep()


while not rospy.is_shutdown() and motion_count < end_step:
    now_s = rospy.Time.now().to_sec() - start_s

    dataset.cal_high_freq()
    dataset.cal_features(1)
    inputs_t = dataset.get_connected_data()
    if inputs_t is not None:
        motion_count += 1
        if mode == "normal":
            output = net(inputs_t)
        elif mode == "custom":
            outpu = net(inputs_t[0], inputs_t[1], inputs_t[2])
        elif mode == "CNNMTRNN":
            # inputs_t = inputs_t[0].to(device), inputs_t[1].to(device)
            output, output_img = net(inputs_t[0], inputs_t[1])
        output = output.detach().numpy()[0]
        normalized_position = output[:7]
        joint_position = dataset.reverse_position(normalized_position)
        if motion_count > start_frame:
            ret = follow_trajectory(
                action_service_name,
                joint_names=JOINT_NAMES,
                positions=joint_position,
                time_from_start=1,
            )
            print(ret)
            print(motion_count)
        outputs.append(output)
        # output_imgs.append(output_img[0])
        cs_states.append(net.cs_state.detach().numpy()[0])
    rate.sleep()
# dataset.save_inputs(outputs, cs_states, output_imgs)
dataset.save_inputs(outputs, cs_states, open_rate, container)
# while not rospy.is_shutdown() and finish_s > now_s:
#     now_s = rospy.Time.now().to_sec() - start_s
#     motion_count += 1
#     dataset.cal_high_freq()
#     if now_s - old_s > output_interval_s:
#         print(motion_count)
#         old_s = now_s
#         dataset.cal_features(motion_count)
#         motion_count = 0
#         inputs_t = dataset.get_connected_data()
#         if inputs_t is not None:
#             output = net(inputs_t)
#             output = output.detach().numpy()[0]
#             position = output[:7]
#             position2 = dataset.reverse_position(position)
#             # print(position2)
#             count += 1
#             # if motion_count > start_frame:
#             follow_trajectory(ac, JOINT_NAMES, position2)
#             outputs.append(output)


# print(count)

# df_output = pd.DataFrame(data=connected_data, columns=header)
# df_output.to_excel(RESULT_DIR + "output{:02}.xlsx".format(j + 1), index=False)
# np_output = outputs.view(-1, dataset[0][0].shape[1]).detach().numpy()
# cs_pca = PCA(n_components=2).fit_transform(cs_states)
# cf_pca = PCA(n_components=2).fit_transform(cf_states)
# header = (
#     get_header("in ")
#     + get_header("out ")
#     + ["cf_pca{}".format(i) for i in range(cf_pca.shape[1])]
#     + ["cs_pca{}".format(i) for i in range(cs_pca.shape[1])]
# )
# df_output = pd.DataFrame(data=connected_data, columns=header)
# df_output.to_excel(RESULT_DIR + "output{:02}.xlsx".format(j + 1), index=False)

"""
while not rospy.is_shutdown() and motion_count < end_step:
    now = rospy.Time.now().to_sec()
    diff = now - start_time
    print(now, diff)
    motion_count += 1
    dataset.cal_high_freq()
    if motion_count % high_freq == 0:
        dataset.cal_features()
        inputs_t = dataset.get_connected_data()
        if inputs_t is not None:
            output = net(inputs_t)
            output = output.detach().numpy()[0]
            position = output[:7]
            position2 = dataset.reverse_position(position)
            if motion_count > start_frame:
                follow_trajectory(
                    action_service_name=ACTION_SERVICE_NAME,
                    joint_names=JOINT_NAMES,
                    positions=position2,
                )
            outputs.append(output)
"""
