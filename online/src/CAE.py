#!/usr/bin/env python3
# coding: utf-8
import torch
# from torchsummary import summary


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 256, 6, 8)


# This is bad name because it use for not only CNN but also FullConnect.
# This class makes this code not to read easily, but this helps to write model.
# Therefore, I use this.
class CNNCell(torch.nn.Module):
    def __init__(
        self,
        in_channels,
        out_channels,
        kernel_size=4,
        stride=2,
        padding=1,
        activate="relu",
        mode="conv",
    ):
        super(CNNCell, self).__init__()
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

    def forward(self, x):
        x = self.nn(x)
        x = self.batchnorm(x)
        x = self.activate(x)
        return x


class CAE(torch.nn.Module):
    def __init__(self):
        super(CAE, self).__init__()
        hidden_dim = 20
        class_num = 6
        self.feature_extract = torch.nn.Sequential(
            # 128*96*3
            CNNCell(3, 32),
            CNNCell(32, 64),
            CNNCell(64, 128),
            CNNCell(128, 256),
        )
        self.attention_extract = torch.nn.Sequential(
            CNNCell(256, 512, 3, 1, 1),
            CNNCell(512, 512, 3, 1, 1),
            CNNCell(512, class_num, 1, 1, 0),
        )
        self.get_attention_map = CNNCell(class_num, 1, 1, 1, 0, "sigmoid")
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
            CNNCell(8 * 6 * 256, 254, mode="linear"),
            CNNCell(254, hidden_dim, activate="sigmoid", mode="linear"),
        )
        self.decoder = torch.nn.Sequential(
            CNNCell(hidden_dim, 254, mode="linear"),
            CNNCell(254, 8 * 6 * 256, mode="linear"),
            ToImg(),
            CNNCell(256, 128, mode="conv_trans"),
            CNNCell(128, 64, mode="conv_trans"),
            CNNCell(64, 32, mode="conv_trans"),
            # torch.nn.Dropout2d(p=0.3),
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            torch.nn.ReLU(),
        )

    def encoder(self, x):
        feature_extracted = self.feature_extract(x)
        attention_extracted = self.attention_extract(feature_extracted)
        self.attention_map = self.get_attention_map(attention_extracted)
        box_class = self.get_class(attention_extracted)  # GAP
        temp = feature_extracted * self.attention_map  # dot
        hidden = self.get_hidden(temp)  # flatten
        return hidden, box_class.view(box_class.size(0), -1)

    def forward(self, x):
        hidden, box_class = self.encoder(x)
        x = self.decoder(hidden)
        return x, box_class


if __name__ == "__main__":
    net = CAE()
    # device = torch.device("cuda:0")
    # net = net.to(device)
    summary(net, (3, 96, 128), device="cpu")
