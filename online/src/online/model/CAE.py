#!/usr/bin/env python3
# coding: utf-8
import torch
from torchsummary import summary
from .cell import Cell


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 256, 6, 8)


class CAE(torch.nn.Module):
    def __init__(self):
        super(CAE, self).__init__()
        hidden_dim = 15  # 20
        self.encoder = torch.nn.Sequential(
            # 128*96*3
            Cell(3, 32),
            Cell(32, 64),
            Cell(64, 128),
            Cell(128, 256),
            torch.nn.Flatten(),
            Cell(8 * 6 * 256, 254, mode="linear"),
            Cell(254, hidden_dim, activate="sigmoid", mode="linear"),
        )
        self.decoder = torch.nn.Sequential(
            Cell(hidden_dim, 254, mode="linear"),
            Cell(254, 8 * 6 * 256, mode="linear"),
            ToImg(),
            Cell(256, 128, mode="conv_trans"),
            Cell(128, 64, mode="conv_trans"),
            Cell(64, 32, mode="conv_trans"),
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            torch.nn.ReLU(),
        )

    def forward(self, x):
        hidden = self.encoder(x)
        x = self.decoder(hidden)
        return x


if __name__ == "__main__":
    net = CAE()
    # device = torch.device("cuda:0")
    # net = net.to(device)
    summary(net, (3, 96, 128), device="cpu")
