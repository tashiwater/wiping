#!/usr/bin/env python
# coding: utf-8

from manager.manager_base import ManagerBase
from model.MTRNN import MTRNN
import torch
import pandas as pd
import numpy as np


class Manager(ManagerBase):
    def set_MTRNN(self):
        in_size, out_size = 30, 30
        self._net = MTRNN(
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs":  self._cs_num},
            tau={"tau_io": 2, "tau_cf": 5, "tau_cs": 10},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/1231/io2cf5cs10/5000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        self._net.init_state(1)

    #     other_cs_path = self._data_dir + "8006result/output05.csv"
    #     df = pd.read_csv(other_cs_path)
    #     io_start = 64
    #     io_num = 50
    #     cf_start = io_start + io_num
    #     cs_start = cf_start + self._cf_num
    #     self.other_io = np.array(
    #         df.iloc[:, io_start: cf_start])
    #     self.other_io = torch.from_numpy(self.other_io.astype(np.float32))
    #     self.other_cf = np.array(
    #         df.iloc[:, cf_start: cs_start])
    #     self.other_cf = torch.from_numpy(self.other_cf.astype(np.float32))
    #     self.other_cs = np.array(
    #         df.iloc[:, cs_start: cs_start + self._cs_num])
    #     self.other_cs = torch.from_numpy(self.other_cs.astype(np.float32))
    #     self._temp_count = 0

    # def add(self):
    #     # print(self._net.cs_state)
    #     if self._temp_count > 128 or self._temp_count < 0:
    #         self._temp_count = -1
    #     self._net.io_state = self.other_io[self._temp_count].unsqueeze(0)
    #     self._net.cf_state = self.other_cf[self._temp_count].unsqueeze(0)
    #     self._net.cs_state = self.other_cs[self._temp_count].unsqueeze(0)
    #     # print(self._net.cs_state)
    #     self._temp_count += 1


if __name__ == "__main__":
    before_scale = [
        # 1228
        # [-0.00045380264974936857, 0.2525317203637283],
        # [-0.4095415449331442, 0.2959554940438517],
        # [-0.4832642188483968, 0.07360053468975894],
        # [-0.8050505611875245, 0.04665259018573553],
        # [-0.00010472880986733824, 0.8230797955827112],
        # [-0.1529780982846586, 0.24397957357299477],
        # [-0.0026004436464077685, 0.5536359703220574],
        # [-9.923999786376953, 17.700000762939453],
        # [-10.510499954223633, 19.344000339508057],
        # [-4.583999872207642, 10.055998802185059],
        # [-6.4599997997283936, 14.13599944114685],
        # [-1.5686665773391724, 3.4993333220481873],
        # [-3.6000000834465027, 3.4500001072883606],
        # [-1.558333396911621, 3.4906667470932007],
        # [0.0, 0.7161290049552917],
        # [0.0, 0.6193548440933228],
        # [0.0, 0.5129032135009766],
        # [0.0, 0.7645161151885986],
        # [-0.029032262042164803, 0.6451613157987595],
        # [-0.03870967775583267, 0.6645161211490631],
        # [-0.029032256454229355, 0.5645161587744951],
        # [-0.03870967775583267, 0.6258064769208431],
        # [-0.43548389966599643, 0.43225806951522827],
        # [-0.6161290286108851, 0.24193552136421204],
        # [-0.4258064776659012, 0.23225805163383484],
        # [-0.48709677439182997, 0.4322580099105835],
        # [-0.10967742651700974, 0.7774193286895752],
        # [-0.08709677122533321, 0.7516129016876221],
        # [-0.06129032373428345, 0.5580645203590393],
        # [-0.032258064951747656, 0.5193548081442714],

        [-0.09466658832071984, 0.12419758270028147],
        [-0.08339182952976021, 0.3512125912640651],
        [-0.24754004934167334, 0.1188394669540154],
        [-0.3391873203884078, 0.3230777923754913],
        [-0.08902924221475353, 0.3653148614738753],
        [-0.09232791676470975, 0.19097391898011812],
        [-0.10159566742391513, 0.041329397219857844],
        [-8.736001968383789, 12.552001953125],
        [-8.814000129699707, 9.047999858856201],
        [-5.123999714851379, 7.823999583721161],
        [-7.435333251953125, 8.170000076293945],
        [-1.085999995470047, 2.7149998247623444],
        [-2.2000001072883606, 1.6500000357627869],
        [-1.8076667785644531, 2.493333578109741],
    ]
    manager = Manager()
    manager.init(before_scale)
    manager.run()
