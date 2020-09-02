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
import cv2
import PIL


class OnlineDataSet:
    def __init__(self, cae, device, high_freq):
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

        self._position_before_scale = [
            [-1.309, 4.451],
            [-2.094, 0.140],
            [-2.880, 2.880],
            [-0.524, 2.269],
            [-2.880, 2.880],
            [-1.396, 1.396],
            [-1.396, 0.087],
        ]
        self._effort_before_scale = [
            [-2, 40],
            [-40, 15],
            [-5, 15],
            [-10, 15],
            [-5, 5],
            [-5, 5],
            [-5, 5],
        ]
        self._last_tactile  = None
        self._last_img = None
        self._connected_data = None

    def cal_high_freq(self):
        if self._last_tactile is None:
            return
        self._tactiles.append(self._last_tactile)
        self._positions.append(self._last_position)
        self._efforts.append(self._last_effort)

    def _get_mean(self, data):
        return np.array(data).mean(axis=0)

    def cal_features(self):
        if self._last_img is None:
            return
        # log
        self.cal_high_freq()
        self._imgs.append(self._last_img)
        # extract feature
        position = self._get_mean(self._positions[-self._high_freq :])
        effort = self._get_mean(self._efforts[-self._high_freq :])
        tactile = self._get_mean(self._tactiles[-self._high_freq :])
        img_feature = self.get_img_feature(self._imgs[-1])

        tactile_before_scale = [[0, 1] for _ in tactile]
        img_before_scale = [[0, 1] for _ in img_feature]

        position = self.normalize(position, self._position_before_scale)
        effort = self.normalize(effort, self._effort_before_scale)
        tactile = self.normalize(tactile, tactile_before_scale)
        img_feature = self.normalize(img_feature, img_before_scale)
        # print(position.shape,effort.shape,tactile.shape,img_feature.shape )
        connected_data = np.hstack(
            [position, effort, tactile,img_feature]
        )
        self._connected_data = torch.tensor(connected_data).float()
    def get_connected_data(self):
        return self._connected_data

    def cb_image(self, data):
        img = self._bridge.imgmsg_to_cv2(data, "rgb8")
        img = cv2.resize(img, dsize=(96,128))
        img = PIL.Image.fromarray(img)
        self._last_img = img

    def TouchSensorCallback(self, data):
        self._last_tactile = data.data

    def ToroboCallback(self, data):
        self._last_position = data.position
        self._last_effort = data.effort

    def get_img_feature(self, img):
        img_tensor = torchvision.transforms.ToTensor()(img)
        img_tensor = torch.unsqueeze(img_tensor,0)
        # img_tensor = img_tensor.to(self._device)
        img_feature = self._cae.encoder(img_tensor)
        # img_feature = torch.zeros(size = (1,20))
        return img_feature.to("cpu").detach().numpy()[0]

    def _each_normalization(self, data, indataRange, outdataRange=[-0.9, 0.9]):
        data = (data - indataRange[0]) / (indataRange[1] - indataRange[0])
        data = data * (outdataRange[1] - outdataRange[0]) + outdataRange[0]
        return data
    def normalize(self, data, before_scale, after_scale = [-0.9,0.9]):
        if np.array(before_scale).ndim ==1:
            before_scales = [before_scale for _ in data]
        else:
            before_scales = before_scale
        if np.array(after_scale).ndim ==1:
            after_scales = [after_scale for _ in data]
        else:
            after_scales = after_scale
        ret =[]
        for d, before, after in zip(data, before_scales, after_scales):
            ret.append(self._each_normalization(d, before, after))
        return ret

    def get_header(self, add_word):
        return (
            [add_word + "position{}".format(i) for i in range(7)]
            + [add_word + "torque{}".format(i) for i in range(7)]
            + [add_word + "tactile{}".format(i) for i in range(16)]
            + [add_word + "image{}".format(i) for i in range(20)]
        )
    
    def reverse_position(self, position):
        return self.normalize(position, [-1, 1], self._position_before_scale)

