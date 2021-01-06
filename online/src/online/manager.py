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
            tau={"tau_io": 2, "tau_cf": 10, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/0102/io2cf10cs30/5000/{}_{}.pth".format(
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

        [-0.02895495263577752, 0.2889218192888432],
        [-0.16065752302498387, 0.406155583197866],
        [-0.4918687009157395, 0.053354713868596595],
        [-0.4755498900530347, 0.3360805432397487],
        [-0.000383961250162157, 0.5336693195233517],
        [-0.42189843554057066, 0.0460417837867529],
        [-0.0505448001188864, 0.3231477003541059],
        [-8.304000854492188, 13.331998825073242],
        [-12.733500480651855, 9.574499607086182],
        [-5.123999714851379, 13.115999460220337],
        [-8.03066611289978, 8.233333110809326],
        [-1.3273332118988037, 2.1720000207424164],
        [-3.75, 3.350000321865082],
        [-2.0570000410079956, 3.553000330924988],
    ]
    manager = Manager()
    manager.init(before_scale)
    manager.run()
