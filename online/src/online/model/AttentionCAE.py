#!/usr/bin/env python3
# coding: utf-8
import torch
from torchsummary import summary
from .cell import Cell


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 256, 6, 8)


class AttentionCAE(torch.nn.Module):
    def __init__(self):
        super(AttentionCAE, self).__init__()
        hidden_dim = 15  # 20
        class_num = 6
        self.feature_extract = torch.nn.Sequential(
            # 128*96*3
            Cell(3, 32),
            Cell(32, 64),
            Cell(64, 128),
            Cell(128, 256),
        )
        self.attention_extract = torch.nn.Sequential(
            Cell(256, 512, 3, 1, 1),
            Cell(512, 512, 3, 1, 1),
            Cell(512, class_num, 1, 1, 0),
        )
        self.get_attention_map = Cell(class_num, 1, 1, 1, 0, "sigmoid")
        self.get_class = torch.nn.Sequential(
            # torch.nn.Dropout2d(p=0.3),
            torch.nn.Conv2d(
                class_num, class_num, 1, stride=1, padding=0
            ),  # ->8*6*class_num
            torch.nn.AvgPool2d(kernel_size=(6, 8)),
            torch.nn.Sigmoid(),
        )
        self.get_hidden = torch.nn.Sequential(
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
            # torch.nn.Dropout2d(p=0.3),
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            torch.nn.ReLU(),
        )

    def encoder(self, x):
        feature_extracted = self.feature_extract(x)
        attention_extracted = self.attention_extract(feature_extracted)
        self.attention_map = self.get_attention_map(attention_extracted)
        # box_class = self.get_class(attention_extracted)  # GAP
        temp = feature_extracted * self.attention_map  # dot
        hidden = self.get_hidden(temp)  # flatten
        return hidden  # , box_class.view(box_class.size(0), -1)

    def forward(self, x):
        hidden = self.encoder(x)
        x = self.decoder(hidden)
        return x


if __name__ == "__main__":
    net = AttentionCAE()
    # device = torch.device("cuda:0")
    # net = net.to(device)
    summary(net, (3, 96, 128), device="cpu")
