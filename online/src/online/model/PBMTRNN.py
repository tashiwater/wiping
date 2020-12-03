#!/usr/bin/env python
# coding:utf-8


import torch
from torchsummary import summary
from .MTRNN import MTRNN
from .cell import Cell


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 64, 12, 16)


class PBMTRNN(MTRNN):  # [TODO]cannot use GPU now
    def __init__(
        self,
        layer_size={"in": 1, "out": 1, "io": 3, "cf": 4, "cs": 5},
        tau={"tau_io": 2, "tau_cf": 5.0, "tau_cs": 70.0},
        open_rate=1,
    ):
        super(PBMTRNN, self).__init__(layer_size, tau, open_rate, torch.nn.Tanh())
        self._pb_dim = 5
        self.encoder = torch.nn.Sequential(
            # 128*96*3
            Cell(3, 16, on_batchnorm=False),  # 64*48
            Cell(16, 32, on_batchnorm=False),  # 32*i24
            Cell(32, 64, on_batchnorm=False),  # 16*12
            torch.nn.Flatten(),
            Cell(16 * 12 * 64, 254, mode="linear", on_batchnorm=False),
            Cell(
                254,
                self._pb_dim,
                mode="linear",
                on_batchnorm=False,
            ),
        )
        self.pb2io = torch.nn.Linear(self._pb_dim, layer_size["io"])
        self.pb2cf = torch.nn.Linear(self._pb_dim, layer_size["cf"])
        self.pb2cs = torch.nn.Linear(self._pb_dim, layer_size["cs"])

    def set_pb(self, img):
        self.pb = self.encoder(img)
        self.pb_io = self.pb2io(self.pb) / self.tau["tau_io"]
        self.pb_cf = self.pb2cf(self.pb) / self.tau["tau_cf"]
        self.pb_cs = self.pb2cs(self.pb) / self.tau["tau_cs"]
        # self.pb_io = self.pb_io.to(self.device)
        # self.pb_cf = self.pb_cf.to(self.device)
        # self.pb_cs = self.pb_cs.to(self.device)

    def forward(self, x):
        if (
            self.last_output is not None and self.open_rate != 1
        ):  # start val is not changed by open_rate
            x = x * self.open_rate + self.last_output * (1 - self.open_rate)

        tau = self.tau["tau_io"]
        temp = (
            (self.io2io(self.io_state) + self.cf2io(self.cf_state) + self.i2io(x)) / tau
            + (1.0 - 1.0 / tau) * self.io_state
            + self.pb_io
        )
        new_io_state = self.activate(temp)

        tau = self.tau["tau_cf"]
        temp = (
            (
                self.cf2cf(self.cf_state)
                + self.cs2cf(self.cs_state)
                + self.io2cf(self.io_state)
            )
            / tau
            + (1.0 - 1.0 / tau) * self.cf_state
            + self.pb_cf
        )
        new_cf_state = self.activate(temp)

        tau = self.tau["tau_cs"]
        temp = (
            (self.cs2cs(self.cs_state) + self.cf2cs(self.cf_state)) / tau
            + (1.0 - 1.0 / tau) * self.cs_state
            + self.pb_cs
        )
        new_cs_state = self.activate(temp)
        self.io_state = new_io_state
        self.cf_state = new_cf_state
        self.cs_state = new_cs_state
        y = self.activate(self.io2o(self.io_state))
        self.last_output = y.detach()
        return y