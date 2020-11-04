#!/usr/bin/env python3
# coding: utf-8
import torch

# This is bad name because it use for not only CNN but also FullConnect.
# This class makes this code not to read easily, but this helps to write model.
# Therefore, I use this.
class Cell(torch.nn.Module):
    def __init__(
        self,
        in_channels,
        out_channels,
        kernel_size=4,
        stride=2,
        padding=1,
        activate="relu",
        mode="conv",
        on_batchnorm=True,
    ):
        super(Cell, self).__init__()
        if mode == "conv":
            self.nn = torch.nn.Conv2d(
                in_channels, out_channels, kernel_size, stride, padding
            )
            self.batchnorm = torch.nn.BatchNorm2d(out_channels)
        elif mode == "linear":
            self.nn = torch.nn.Linear(in_channels, out_channels)
            self.batchnorm = torch.nn.BatchNorm1d(out_channels)
        elif mode == "conv_trans":
            self.nn = torch.nn.ConvTranspose2d(
                in_channels, out_channels, kernel_size, stride, padding
            )
            self.batchnorm = torch.nn.BatchNorm2d(out_channels)

        if activate == "relu":
            self.activate = torch.nn.ReLU()
        elif activate == "sigmoid":
            self.activate = torch.nn.Sigmoid()
        elif activate == "tanh":
            self.activate = torch.nn.Tanh()
        elif activate == "softmax":
            self.activate = torch.nn.Softmax(dim=1)

        self._on_batchnorm = on_batchnorm

    def forward(self, x):
        x = self.nn(x)
        if self._on_batchnorm:
            x = self.batchnorm(x)
        x = self.activate(x)
        return x