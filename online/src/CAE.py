#!/usr/bin/env python3
# coding: utf-8
import torch


class ToImg(torch.nn.Module):
    def forward(self, x):
        n, _ = x.shape
        return x.reshape(n, 256, 6, 8)


class Nothing(torch.nn.Module):
    def __init__(self, num):
        super().__init__()
        self._str = str(num)

    def forward(self, x):
        print(self._str)
        return x


class CAEwithAttention(torch.nn.Module):
    def __init__(self):
        super(CAEwithAttention, self).__init__()
        hidden_dim = 20
        class_num = 6
        self.activate = torch.nn.ReLU(True)
        self.feature_extract = torch.nn.Sequential(
            # 128*96*3
            torch.nn.Conv2d(3, 32, 4, stride=2, padding=1),  # ->64*48*32
            torch.nn.BatchNorm2d(32),
            self.activate,
            torch.nn.Conv2d(32, 64, 4, stride=2, padding=1),  # ->32*24*64
            torch.nn.BatchNorm2d(64),
            self.activate,
            torch.nn.Conv2d(64, 128, 4, stride=2, padding=1),  # ->16*12*128
            torch.nn.BatchNorm2d(128),
            self.activate,
            torch.nn.Conv2d(128, 256, 4, stride=2, padding=1),  # ->8*6*256
            torch.nn.BatchNorm2d(256),
            self.activate,
        )
        self.attention_extract = torch.nn.Sequential(
            torch.nn.Conv2d(256, 512, 3, stride=1, padding=1),  # ->8*6*512
            torch.nn.BatchNorm2d(512),
            self.activate,
            torch.nn.Conv2d(512, 512, 3, stride=1, padding=1),  # ->8*6*512
            torch.nn.BatchNorm2d(512),
            self.activate,
            torch.nn.Conv2d(512, class_num, 1, stride=1, padding=0),  # ->8*6*class_num
            torch.nn.BatchNorm2d(class_num),
            self.activate,
        )
        self.get_attention_map = torch.nn.Sequential(
            torch.nn.Conv2d(class_num, 1, 1, stride=1, padding=0),  # ->8*6*1
            torch.nn.BatchNorm2d(1),
            torch.nn.Sigmoid(),
        )
        self.get_class = torch.nn.Sequential(
            torch.nn.Conv2d(
                class_num, class_num, 1, stride=1, padding=0
            ),  # ->8*6*class_num
            torch.nn.AvgPool2d(kernel_size=(6, 8)),
            torch.nn.Sigmoid(),
        )
        self.get_hidden = torch.nn.Sequential(
            torch.nn.Flatten(),
            torch.nn.Linear(8 * 6 * 256, 254),
            torch.nn.BatchNorm1d(254),
            self.activate,
            torch.nn.Linear(254, hidden_dim),
            torch.nn.BatchNorm1d(hidden_dim),
            torch.nn.Sigmoid(),
        )
        self.decoder = torch.nn.Sequential(
            torch.nn.Linear(hidden_dim, 254),
            torch.nn.BatchNorm1d(254),
            self.activate,
            torch.nn.Linear(254, 8 * 6 * 256),
            torch.nn.BatchNorm1d(8 * 6 * 256),
            self.activate,
            ToImg(),
            torch.nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(128),
            self.activate,
            torch.nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(64),
            self.activate,
            torch.nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(32),
            self.activate,
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            self.activate,
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
        return [x, box_class]


class CAE(torch.nn.Module):
    def __init__(self):
        super(CAE, self).__init__()
        hidden_dim = 20
        self.activate = torch.nn.ReLU(True)
        self.encoder = torch.nn.Sequential(
            # 128*96*3
            torch.nn.Conv2d(3, 32, 4, stride=2, padding=1),  # ->64*48*32
            torch.nn.BatchNorm2d(32),
            self.activate,
            torch.nn.Conv2d(32, 64, 4, stride=2, padding=1),  # ->32*24*64
            torch.nn.BatchNorm2d(64),
            self.activate,
            torch.nn.Conv2d(64, 128, 4, stride=2, padding=1),  # ->16*12*128
            torch.nn.BatchNorm2d(128),
            self.activate,
            torch.nn.Conv2d(128, 256, 4, stride=2, padding=1),  # ->8*6*256
            torch.nn.BatchNorm2d(256),
            self.activate,
            torch.nn.Flatten(),
            torch.nn.Linear(8 * 6 * 256, 254),
            torch.nn.BatchNorm1d(254),
            self.activate,
            torch.nn.Linear(254, hidden_dim),
            torch.nn.BatchNorm1d(hidden_dim),
            torch.nn.Sigmoid(),
        )
        self.decoder = torch.nn.Sequential(
            torch.nn.Linear(hidden_dim, 254),
            torch.nn.BatchNorm1d(254),
            self.activate,
            torch.nn.Linear(254, 8 * 6 * 256),
            torch.nn.BatchNorm1d(8 * 6 * 256),
            self.activate,
            ToImg(),
            torch.nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(128),
            self.activate,
            torch.nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(64),
            self.activate,
            torch.nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(32),
            self.activate,
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            self.activate,
        )

    def forward(self, x):
        image = x
        image2 = self.encoder(image)
        x = self.decoder(image2)

        return x


if __name__ == "__main__":
    net = Net()
    print(net)


class Net(torch.nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        hidden_dim = 20
        self.encoder = torch.nn.Sequential(
            # 128*96*3
            # Nothing(1),
            torch.nn.Conv2d(3, 32, 4, stride=2, padding=1),  # ->64*48*32
            torch.nn.BatchNorm2d(32),
            torch.nn.ReLU(),
            torch.nn.Conv2d(32, 64, 4, stride=2, padding=1),  # ->32*24*64
            torch.nn.BatchNorm2d(64),
            torch.nn.ReLU(),
            torch.nn.Conv2d(64, 128, 4, stride=2, padding=1),  # ->16*12*128
            torch.nn.BatchNorm2d(128),
            torch.nn.ReLU(),
            torch.nn.Conv2d(128, 256, 4, stride=2, padding=1),  # ->8*6*256
            torch.nn.BatchNorm2d(256),
            torch.nn.ReLU(),
            torch.nn.Flatten(),
            # Nothing(2),
            torch.nn.Linear(8 * 6 * 256, 254),
            torch.nn.BatchNorm1d(254),
            torch.nn.ReLU(),
            # Nothing(3),
            torch.nn.Linear(254, hidden_dim),
            # Nothing(4),
            torch.nn.BatchNorm1d(hidden_dim),
            # Nothing(5),
            torch.nn.Sigmoid(),
        )
        self.decoder = torch.nn.Sequential(
            torch.nn.Linear(hidden_dim, 254),
            torch.nn.BatchNorm1d(254),
            torch.nn.ReLU(),
            torch.nn.Linear(254, 8 * 6 * 256),
            torch.nn.BatchNorm1d(8 * 6 * 256),
            torch.nn.ReLU(),
            ToImg(),
            torch.nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(128),
            torch.nn.ReLU(),
            torch.nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(64),
            torch.nn.ReLU(),
            torch.nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
            torch.nn.BatchNorm2d(32),
            torch.nn.ReLU(),
            torch.nn.ConvTranspose2d(32, 3, 4, stride=2, padding=1),
            torch.nn.ReLU(),
        )

    def forward(self, x):
        image = x
        image2 = self.encoder(image)
        x = self.decoder(image2)

        return x


if __name__ == "__main__":
    net = Net()
    print(net)
