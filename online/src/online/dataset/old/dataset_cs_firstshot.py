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

        self._cae = cae
        # self._cae.to(device)
        self._cae.eval()
        self._device = device

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
            [1.6117068753543122, 2.1200514637028007],
            [-0.6368531630633356, 0.11388273535710568],
            [-0.5634097049436209, 0.19519762380936195],
            [0.7681368847920997, 1.688274487748799],
            [0.16900022467955014, 0.9541540264022268],
            [-0.8862432854471518, 0.29183651594388066],
            [-1.2531987873105703, -0.5639856801083778],
        ]
        self._effort_before_scale = [
            [20.052000045776367, 58.91999435424805],
            [-29.796001434326172, 13.532999992370605],
            [-5.3279995918273935, 25.24799728393555],
            [-9.5, 18.227333068847656],
            [-3.86133337020874, 2.5339999198913574],
            [-9.450000762939453, 4.800000190734863],
            [-2.119333267211914, 8.414999961853027],
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

        img_feature = self.get_img_feature(self._imgs[0])

        tactile_before_scale = [[0.0, 0.7548387050628662],
                                [0.0, 0.72258061170578],
                                [0.0, 0.6354838609695435],
                                [0.0, 0.7935483455657959],
                                [0.0, 0.6645161509513855],
                                [0.0, 0.7064515948295593],
                                [0.0, 0.6419354677200317],
                                [0.0, 0.6870967745780945],
                                [0.0, 0.7161290049552917],
                                [0.0, 0.6580645442008972],
                                [0.0, 0.5193548202514648],
                                [0.0, 0.7161290049552917],
                                [0.0, 0.79677414894104],
                                [0.0, 0.8129032254219055],
                                [0.0, 0.6451613306999207],
                                [0.0, 0.5903225541114807], ]
        img_before_scale = [[0.0, 0.4972033500671387],
                            [0.0, 0.4519493579864502],
                            [0.0, 0.4811547100543976],
                            [0.0, 0.40289184451103205],
                            [0.0, 0.4973227083683014],
                            [0.008948994800448418, 0.44961935281753534],
                            [0.0035407552495598797, 0.4977116882801056],
                            [0.0, 0.5266909599304199],
                            [0.0, 0.4620552659034728],
                            [0.0, 0.4891946613788604],
                            [0.0, 0.3642278611660004],
                            [0.0, 0.4532333612442017],
                            [0.0, 0.5029321908950806],
                            [0.05297283828258514, 0.3900356590747833],
                            [0.0, 0.4649696946144104], ]

        position = self.normalize(position, self._position_before_scale)
        effort = self.normalize(effort, self._effort_before_scale)
        tactile = self.normalize(tactile, tactile_before_scale)
        img_feature = self.normalize(img_feature, img_before_scale)
        if self._mode == "custom":
            normalized_tactile = []
            for t in tactiles:
                normalized_tactile.append(
                    self.normalize(t, tactile_before_scale))
            motor = torch.from_numpy(np.hstack([position, effort])).float()
            img = torch.from_numpy(img_feature).float()
            tactiles = torch.from_numpy(normalized_tactile).float()
            self._custom_data = [motor, tactiles, img]

        # print(position.shape,effort.shape,tactile.shape,img_feature.shape )
        connected_data = np.hstack([position, effort, tactile, img_feature])
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
            + [add_word + "image{}".format(i) for i in range(15)]
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

    def save_inputs(self, outputs, cs_states, add_word):
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
        predict_img_dir = log_dir + "predict_img/" + nowstr
        os.makedirs(input_img_dir)
        os.makedirs(output_img_dir)
        os.makedirs(predict_img_dir)
        for i, img in enumerate(self._imgs):
            img.save(input_img_dir + "/{:03d}.jpg".format(i))
            self._decoded_datas[i].save(
                output_img_dir + "/{:03d}.jpg".format(i))

        # add
        df_output = pd.DataFrame(
            data=outputs, columns=self.get_header("out_")[:len(outputs[0])])

        filename = log_dir + "output/" + nowstr + "output.csv"
        # df.to_csv(filename, index=False)
        df_cs = pd.DataFrame(data=cs_states, columns=[
            "cs_states{}".format(i) for i in range(len(cs_states[0]))])

        df_concat = pd.concat([df_input, df_output, df_cs], axis=1, sort=False)
        filename = log_dir + "output/" + nowstr + ".csv"
        df_concat.to_csv(filename, index=False)

        np_outputs = np.array(outputs)
        tensor_outputs = torch.from_numpy(
            np_outputs.astype(np.float32)).clone()
        imgs = self._cae.decoder(tensor_outputs[:, 30:])
        for j, img in enumerate(imgs.cpu()):
            torchvision.utils.save_image(
                img, predict_img_dir + "{:03d}.png".format(j))
        # #DECODE OUTPUT
        # img = self._cae.decoder(img_feature)
        # rgb = torchvision.transforms.functional.to_pil_image(img[0], "RGB")
        # self._decoded_datas.append(rgb)
