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
        in_size, out_size = 45, 45
        self._net = MTRNN(
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs":  self._cs_num},
            tau={"tau_io": 2, "tau_cf": 10, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/0106/all/5000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        self._net.init_state(1)

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

    # def set_addword(self):
    #     return "online_cf{}_cs{}_type{:02d}_open{:02d}_".format(
    #         self._cf_num, self._cs_num, self._container, int(self._open_rate*10))


if __name__ == "__main__":
    before_scale = [
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
    manager.set_cae("CAE/0106/all/20210113_203023_2000.pth")
    manager.run()
