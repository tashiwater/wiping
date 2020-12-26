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
    def set_before_scale(self, val):
        self._position_before_scale = val[:7]
        self._effort_before_scale = val[7:]

    def __init__(self):
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

        self._position_max = [
            [-1.309, 4.451],
            [-2.094, 0.140],
            [-2.880, 2.880],
            [-0.524, 2.269],
            [-2.880, 2.880],
            [-1.396, 1.396],
            [-1.396, 0.087],
        ]

        self._tactile_before_scale = [[0, 1] for _ in range(16)]
        self._last_tactile = None
        self._last_img = None
        self._connected_data = None
        self._connected_datas = []
        self._decoded_datas = []
        self.dsize = 5
        self._img_size = (128, 96)
        self._tactile_frame_num = 5
        self.is_start = False

    def start(self):
        self.is_start = True
        self._start_val = [self._last_position,
                           self._last_effort, self._last_tactile]

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
        position = self._get_mean(
            self._positions[-high_freq:]) - self._start_val[0]
        effort = self._get_mean(
            self._efforts[-high_freq:]) - self._start_val[1]
        tactile = self._get_mean(
            self._tactiles[-high_freq:]) - self._start_val[2]

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
        if self.is_start:
            self._tactiles.append(self._last_tactile)

    def ToroboCallback(self, data):
        self._last_position = data.position
        self._last_effort = data.effort
        if self.is_start:
            self._positions.append(self._last_position)
            self._efforts.append(self._last_effort)

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
        for i in range(len(real_position)):
            real_position[i] += self._start_val[0][i]
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
        nowstr = add_word + now.strftime("%Y%m%d_%H%M%S")
        # filename = log_dir + "input/" + nowstr + ".csv"
        # df.to_csv(filename, index=False)

        input_img_dir = log_dir + "input_img/" + nowstr
        output_img_dir = log_dir + "decoded_img/" + nowstr
        os.mkdir(input_img_dir)
        # os.mkdir(output_img_dir)
        # fourcc = cv2.VideoWriter_fourcc(*'H264')
        # mp4path = input_img_dir + "/"+add_word + ".mp4"
        # fps = 10
        # video = cv2.VideoWriter(mp4path, fourcc, fps, self._imgs[0].size)
        gif = []
        for i, img in enumerate(self._imgs):
            img.save(input_img_dir + "/{:03d}.jpg".format(i))
        #     cv2img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        #     video.write(cv2img)
        # video.release()
            gif.append(img.convert("P", palette=PIL.Image.ADAPTIVE))
        gif[0].save(input_img_dir + "/"+add_word + '.gif',
                    save_all=True, optimize=False, append_images=gif[1:], loop=True, duration=100)
        # self._decoded_datas[i].save(
        #     output_img_dir + "/{:03d}.jpg".format(i))

        # add
        header = self.get_header("out_")
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
