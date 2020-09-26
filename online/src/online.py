#!/usr/bin/env python
# coding: utf-8

import os
import torch
import torchvision
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
# from sklearn.decomposition import PCA
import rospy

from mydataset import OnlineDataSet as MyDataSet
from MTRNN import MTRNN
from CAE import CAE as CAE
from torobo_func import follow_trajectory

rospy.init_node("online")
ACTION_SERVICE_NAME = "/torobo/arm_controller/follow_joint_trajectory"
JOINT_NAMES = ["arm/joint_" + str(i)
               for i in range(1, 8)]  # from joint_1 to joint_8
TIME_INTERVAL = 1

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = CURRENT_DIR + "/../data/"
RESULT_DIR = DATA_DIR + "result/"
MODEL_DIR = DATA_DIR + "model/"

cae = CAE()
cae_path = MODEL_DIR + "CAE/argumentation1/20200924_210301_1500.pth"
checkpoint = torch.load(cae_path)
cae.load_state_dict(checkpoint["model"])

high_freq = 1
dataset = MyDataSet(cae, device=torch.device("cuda:0"), high_freq=high_freq)

in_size = 50
net = MTRNN(
    layer_size={"in": in_size, "out": in_size, "io": 34, "cf": 200, "cs": 15},
    tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 70},
    open_rate=1,
)
model_path = MODEL_DIR + "MTRNN/model/default/20200925_095956_14500.pth"
checkpoint = torch.load(model_path)
net.load_state_dict(checkpoint["model"])

print(net)
net.eval()
net.init_state(1)
alltype_cs = []
outputs = []
io_states = []
cf_states = []
cs_states = []
hz = 20
rate = rospy.Rate(hz * high_freq)
end_step = 30*hz * high_freq
motion_count = 0
start_frame = +5  # 10 * high_freq

while not rospy.is_shutdown() and motion_count < end_step:
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
            print(position2)
            if motion_count > start_frame:
                follow_trajectory(
                    action_service_name=ACTION_SERVICE_NAME,
                    joint_names=JOINT_NAMES,
                    positions=position2,
                )
            outputs.append(output)

            # io_states.append(net.io_state.view(-1).detach().numpy())
            # cf_states.append(net.cf_state.view(-1).detach().numpy())
            # cs_states.append(net.cs_state.view(-1).detach().numpy())
            # alltype_cs += cs_states
            # io_states = np.array(io_states)
            # cf_states = np.array(cf_states)
            # cs_states = np.array(cs_states)

    rate.sleep()

dataset.save_inputs()
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
