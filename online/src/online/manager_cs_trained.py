#!/usr/bin/env python
# coding: utf-8

from manager.manager_base import ManagerBase
from model.MTRNN_cs import MTRNN
import torch
import pandas as pd
import numpy as np


class Manager(ManagerBase):
    def set_MTRNN(self):
        in_size, out_size = 30, 30
        self._net = MTRNN(
            12,
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs":  self._cs_num},
            tau={"tau_io": 2, "tau_cf": 10, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/0106/cs2/cf10cs30/2000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        # cs
        each_container = 3
        num = self._container*each_container
        self._net.init_state(1, self._net.cs0[num:num+1])

        # self._net.init_state(1)

    #     other_cs_path = self._data_dir + "result/output10.csv"
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

    #     self.other_in = np.array(df.iloc[:, :in_size])
    #     self.other_in = torch.from_numpy(self.other_in.astype(np.float32))
    #     self._temp_count = 0

    # def add(self):
    #     # print(self._net.cs_state)
    #     if self._temp_count > 138 or self._temp_count < 0:
    #         self._temp_count = -2
    #     # self._net.io_state = self.other_io[self._temp_count].unsqueeze(0)
    #     # self._net.cf_state = self.other_cf[self._temp_count].unsqueeze(0)
    #     self._net.cs_state = self.other_cs[self._temp_count].unsqueeze(0)
    #     # print(self._net.cs_state)
    #     self._temp_count += 1

    # def temp4offline(self, inputs_t):
    #     if self._temp_count > 138:
    #         self._temp_count = -1
    #     inputs_t2 = self.other_in[self._temp_count].unsqueeze(0)
    #     # print(self._net.cs_state)
    #     self._temp_count += 1
    #     return self._net(inputs_t2)

    def set_addword(self):
        return "cs0_cf{}_cs{}_type{:02d}_open{:02d}_".format(
            self._cf_num, self._cs_num, self._container, int(self._open_rate*10))


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
        [-0.18788481648259991, 0.042219492235189726],
        [-0.2643824543194445, 0.35691982870388905],
        [-0.5818403877102672, 0.03354522767509434],
        [-0.16716768688870987, 0.4605400478332392],
        [-0.028553580969988945, 0.7077135546627552],
        [-0.4690223324184392, 0.00034907383988214136],
        [-0.08326965701475286, 0.2065073698791423],
        [-9.911998748779297, 12.623998641967773],
        [-13.767000198364258, 6.922499656677246],
        [-2.783999979496002, 9.923999547958374],
        [-10.93133282661438, 6.764000415802002],
        [-1.6893332302570343, 1.7496666014194489],
        [-3.9500001668930054, 2.849999964237213],
        [-1.9946666955947876, 2.4310001134872437],
    ]
    manager = Manager()
    manager.init(before_scale, 200)
    manager.run()
