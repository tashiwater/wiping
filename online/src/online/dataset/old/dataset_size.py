#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import torch
import torchvision
import numpy as np
import pandas as pd
import cv2
import PIL
import datetime
import os
from crop import random_crop_image


class OnlineDataSet:
    def __init__(self, cae, device, high_freq, mode="normal"):
        rospy.Subscriber("/image_raw", Image, self.cb_image)
        rospy.Subscriber(
            "/touchence/sensor_data", Float32MultiArray, self.TouchSensorCallback
        )
        rospy.Subscriber(
            "/torobo/joint_state_server/joint_states", JointState, self.ToroboCallback
        )
        self._bridge = CvBridge()
        self._imgs = []
        self._tactiles = []
        self._positions = []
        self._efforts = []
        self._high_freq = high_freq

        # self._cae = cae
        # # self._cae.to(device)
        # self._cae.eval()
        # self._device = device

        self._position_max = [
            [-1.309, 4.451],
            [-2.094, 0.140],
            [-2.880, 2.880],
            [-0.524, 2.269],
            [-2.880, 2.880],
            [-1.396, 1.396],
            [-1.396, 0.087],
        ]
        self._position_before_scale = [
            [1.7873218429046445, 2.058947496177859],
            [-0.6268000628434227, 0.005986479242794388],
            [-0.35454616986905824, 0.02745402833492248],
            [0.8632747860467361, 1.5960337728650325],
            [0.21390754553441596, 0.7578219292580048],
            [-0.16929693411449862, 0.2787116258541295],
            [-1.3753195038347534, -1.0480527588016495],
        ]
        self._effort_before_scale = [
            [21.323997497558594, 43.88399887084961],
            [-12.402000427246096, 15.775500297546387],
            [-5.519999504089356, 13.943999290466307],
            [-6.23199987411499, 9.94333267211914],
            [-1.93066668510437, 2.7149999141693115],
            [-7.15000057220459, 1.900000095367432],
            [-1.807666778564453, 5.485333442687988],
        ]
        self._tactile_before_scale = [
            [0.0, 0.7419354319572449],
            [0.0, 0.6354838609695435],
            [0.0, 0.5774193406105042],
            [0.0, 0.745161235332489],
            [0.0, 0.699999988079071],
            [0.0032258064020425077, 0.6935483813285828],
            [0.006451612804085015, 0.625806450843811],
            [0.0, 0.7032257914543152],
            [0.0, 0.7290322184562683],
            [0.0, 0.6806451678276062],
            [0.0032258064020425077, 0.6064516305923462],
            [0.0, 0.7548387050628662],
            [0.0, 0.8096774220466614],
            [0.0, 0.8258064389228821],
            [0.0, 0.6645161509513855],
            [0.0, 0.5870967507362366],
        ]
        self._last_tactile = None
        self._last_img = None
        self._connected_data = None
        self._connected_datas = []
        self._decoded_datas = []
        self.dsize = 5
        self._img_size = (128, 96)
        self._tactile_frame_num = 5
        self._mode = mode

    def cal_high_freq(self):
        if self._last_tactile is None:
            return
        self._tactiles.append(self._last_tactile)
        self._positions.append(self._last_position)
        self._efforts.append(self._last_effort)

    def _get_mean(self, data):
        return np.array(data).mean(axis=0)

    def cal_features(self, high_freq):
        if self._last_img is None:
            return
        # log
        self._imgs.append(self._last_img)
        # extract feature
        position = self._get_mean(self._positions[-high_freq:])
        effort = self._get_mean(self._efforts[-high_freq:])
        tactile = self._get_mean(self._tactiles[-high_freq:])
        if self._mode == "custom":
            tactiles = self._tactiles[-self._tactile_frame_num:]

        # img_feature = self.get_img_feature(self._imgs[-1])

        # img_before_scale = [[-0.9, 0.9] for _ in img_feature]

        position = self.normalize(position, self._position_before_scale)
        effort = self.normalize(effort, self._effort_before_scale)
        tactile = self.normalize(tactile, self._tactile_before_scale)
        # img_feature = self.normalize(img_feature, img_before_scale)
        connected_data = np.hstack([position, effort, tactile])
        self._connected_data = torch.tensor(
            connected_data).float().unsqueeze(0)
        self._connected_datas.append(connected_data)

    def get_custom_data(self):
        return self._custom_data

    def get_connected_data(self):
        return self._connected_data

    def cb_image(self, data):
        img = self._bridge.imgmsg_to_cv2(data, "rgb8")
        img = PIL.Image.fromarray(img)
        img = img.resize(
            (self._img_size[0] + self.dsize, self._img_size[1] + self.dsize)
        )
        self._last_img = img

    def TouchSensorCallback(self, data):
        self._last_tactile = data.data

    def ToroboCallback(self, data):
        self._last_position = data.position
        self._last_effort = data.effort

    def get_img_feature(self, img):
        img = random_crop_image(img, self._img_size, test=True)
        img_tensor = torchvision.transforms.ToTensor()(img)
        img_tensor = torch.unsqueeze(img_tensor, 0)
        # img_tensor = img_tensor.to(self._device)
        # img_feature, box_class = self._cae.encoder(img_tensor)
        img_feature = self._cae.encoder(img_tensor)
        # val, class_num = torch.max(box_class, 1)
        # img_feature = torch.zeros(size = (1,20))
        img = self._cae.decoder(img_feature)
        rgb = torchvision.transforms.functional.to_pil_image(img[0], "RGB")
        self._decoded_datas.append(rgb)
        return img_feature.to("cpu").detach().numpy()[0]

    def _each_normalization(self, data, indataRange, outdataRange=[-0.9, 0.9]):
        data = (data - indataRange[0]) / (indataRange[1] - indataRange[0])
        data = data * (outdataRange[1] - outdataRange[0]) + outdataRange[0]
        return data

    def normalize(self, data, before_scale, after_scale=[-0.9, 0.9]):
        if np.array(before_scale).ndim == 1:
            before_scales = [before_scale for _ in data]
        else:
            before_scales = before_scale
        if np.array(after_scale).ndim == 1:
            after_scales = [after_scale for _ in data]
        else:
            after_scales = after_scale
        ret = []
        for d, before, after in zip(data, before_scales, after_scales):
            ret.append(self._each_normalization(d, before, after))
        return ret

    def get_header(self, add_word=""):
        return (
            [add_word + "position{}".format(i) for i in range(7)]
            + [add_word + "torque{}".format(i) for i in range(7)]
            + [add_word + "tactile{}".format(i) for i in range(16)]
            # + [add_word + "image{}".format(i) for i in range(15)]
        )

    def reverse_position(self, position):
        real_position = self.normalize(
            position, [-0.9, 0.9], self._position_before_scale
        )
        ret = []
        for min_max, p in zip(self._position_max, real_position):
            p_cliped = min(max(min_max[0], p), min_max[1])
            ret.append(p_cliped)
        return ret

    def save_inputs(self, outputs, cf_states, cs_states, add_word):
        data_dir = "/home/assimilation/TAKUMI_SHIMIZU/wiping_ws/src/wiping/online/data/"

        log_dir = data_dir + "log/"

        df_input = pd.DataFrame(data=self._connected_datas,
                                columns=self.get_header("in_"))

        now = datetime.datetime.now()
        nowstr = now.strftime("%Y%m%d_%H%M%S") + add_word
        # filename = log_dir + "input/" + nowstr + ".csv"
        # df.to_csv(filename, index=False)

        input_img_dir = log_dir + "input_img/" + nowstr
        output_img_dir = log_dir + "decoded_img/" + nowstr
        os.mkdir(input_img_dir)
        os.mkdir(output_img_dir)
        for i, img in enumerate(self._imgs):
            img.save(input_img_dir + "/{:03d}.jpg".format(i))
            # self._decoded_datas[i].save(
            #     output_img_dir + "/{:03d}.jpg".format(i))

        # add
        header = self.get_header(
            "out_") + ["size{}".format(i) for i in range(2)]
        df_output = pd.DataFrame(data=outputs, columns=header)

        filename = log_dir + "output/" + nowstr + "output.csv"
        # df.to_csv(filename, index=False)
        df_cf = pd.DataFrame(data=cf_states, columns=[
            "cf_states{}".format(i) for i in range(len(cf_states[0]))])
        df_cs = pd.DataFrame(data=cs_states, columns=[
            "cs_states{}".format(i) for i in range(len(cs_states[0]))])

        df_concat = pd.concat(
            [df_input, df_output, df_cf, df_cs], axis=1, sort=False)
        filename = log_dir + "output/" + nowstr + ".csv"
        df_concat.to_csv(filename, index=False)

        # #DECODE OUTPUT
        # img = self._cae.decoder(img_feature)
        # rgb = torchvision.transforms.functional.to_pil_image(img[0], "RGB")
        # self._decoded_datas.append(rgb)
