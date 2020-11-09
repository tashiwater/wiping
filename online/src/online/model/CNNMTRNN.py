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


class CNNMTRNN(torch.nn.Module):  # [TODO]cannot use GPU now
    def __init__(
        self,
        layer_size={"in": 1, "out": 1, "io": 3, "cf": 4, "cs": 5},
        tau={"tau_io": 2, "tau_cf": 5.0, "tau_cs": 70.0},
        open_rate=1,
    ):
        super(CNNMTRNN, self).__init__()
        self.open_rate = open_rate
        self.mtrnn = MTRNN(layer_size, tau, 1, torch.nn.Tanh())
        self._hidden_dim = 15

        self.encoder = torch.nn.Sequential(
            # 128*96*3
            Cell(3, 16, on_batchnorm=False),  # 64*48
            Cell(16, 32, on_batchnorm=False),  # 32*i24
            Cell(32, 64, on_batchnorm=False),  # 16*12
            torch.nn.Flatten(),
            Cell(16 * 12 * 64, 254, mode="linear", on_batchnorm=False),
            Cell(
                254,
                self._hidden_dim,
                activate="tanh",
                mode="linear",
                on_batchnorm=False,
            ),
        )
        self.decoder = torch.nn.Sequential(
            Cell(self._hidden_dim, 254, mode="linear", on_batchnorm=False),
            Cell(254, 16 * 12 * 64, mode="linear", on_batchnorm=False),
            ToImg(),
            Cell(64, 32, mode="conv_trans", on_batchnorm=False),
            Cell(32, 16, mode="conv_trans", on_batchnorm=False),
            Cell(16, 3, mode="conv_trans", activate="relu", on_batchnorm=False),
        )
        self.last_img = None

    def init_state(self, batch_size):
        self.mtrnn.init_state(batch_size)
        self.last_img = None

    def forward(self, x, img):

        if self.mtrnn.last_output is not None:
            x = x * self.open_rate + self.mtrnn.last_output[:, : -self._hidden_dim] * (
                1 - self.open_rate
            )
            img = img * self.open_rate + self.last_img * (1 - self.open_rate)

        hidden = self.encoder(img)
        temp = torch.cat([x, hidden], axis=1)
        ret = self.mtrnn(temp)
        img = self.decoder(ret[:, -self._hidden_dim :])
        # self.last_motor = ret[:, : -self._hidden_dim]
        self.last_img = img.detach()
        return ret[:, : -self._hidden_dim], img
