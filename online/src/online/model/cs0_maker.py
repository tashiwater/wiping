#!/usr/bin/env python3
# coding: utf-8
import torch
from torchsummary import summary
from .cell import Cell


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 256, 6, 8)


class Cs0Maker(torch.nn.Module):
    def __init__(self, hidden_dim=10):
        super(Cs0Maker, self).__init__()
        self.encoder = torch.nn.Sequential(
            # 128*96*3
            Cell(3, 8, on_batchnorm=False),  # 64*48
            Cell(8, 16, on_batchnorm=False),  # 32*i24
            Cell(16, 32, on_batchnorm=False),  # 16*12
            torch.nn.Flatten(),
            Cell(16 * 12 * 32, 128, mode="linear", on_batchnorm=False),
            Cell(128, 64, mode="linear", on_batchnorm=False),
            torch.nn.Linear(64, hidden_dim),
        )

    def forward(self, x):
        x = self.encoder(x)
        return x


if __name__ == "__main__":
    net = Cs0Maker()
    # device = torch.device("cuda:0")
    # net = net.to(device)
    summary(net, (3, 96, 128), device="cpu")
