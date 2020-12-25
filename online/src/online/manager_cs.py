#!/usr/bin/env python
# coding: utf-8

from manager.manager_base import ManagerBase
from model.MTRNN_cs import MTRNN
import torch


class Manager(ManagerBase):
    def set_MTRNN(self):
        in_size, out_size = 30, 30
        self._net = MTRNN(
            20,
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs": self._cs_num},
            tau={"tau_io": 2, "tau_cf": 10, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/1223/cs/5000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        each_container = 4
        num = self._container*each_container
        self._net.init_state(1, self._net.cs0[num:num+1])


if __name__ == "__main__":
    before_scale = [
        [-0.09466658832071984, 0.06054550264435199],
        [-0.1813222549244265, 0.35081116966400616],
        [-0.24754004934167334, 0.1188394669540154],
        [-0.19846142759379481, 0.284383925949403],
        [-0.08902924221475353, 0.3653148614738753],
        [-0.17163567849131955, 0.08040731436167627],
        [-0.10159566742391513, 0.04415680877016781],
        [-7.75200080871582, 10.139999389648438],
        [-8.677500247955322, 7.293000221252441],
        [-5.123999714851379, 7.823999583721161],
        [-6.371333122253418, 5.8266661167144775],
        [-1.085999920964241, 2.1720000207424164],
        [-2.2000001072883606, 1.6500000357627869],
        [-1.8076667785644531, 2.2440000772476196],
    ]
    manager = Manager()
    manager.init(before_scale)
    manager.run()
