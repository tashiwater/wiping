#!/usr/bin/env python
# coding:utf-8

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class MTRNN(nn.Module):  # [TODO]cannot use GPU now
    def __init__(
        self,
        layer_size={"in": 1, "out": 1, "io": 3, "cf": 4, "cs": 5},
        tau={"tau_io": 2, "tau_cf": 5.0, "tau_cs": 70.0},
        open_rate=1,
        activate=torch.nn.Tanh(),
    ):
        super(MTRNN, self).__init__()
        self.layer_size = layer_size
        self.tau = tau
        self.open_rate = open_rate
        self.last_output = None
        self.i2io = nn.Linear(self.layer_size["in"], self.layer_size["io"])
        self.io2o = nn.Linear(self.layer_size["io"], self.layer_size["out"])
        self.io2io = nn.Linear(self.layer_size["io"], self.layer_size["io"])
        self.io2cf = nn.Linear(self.layer_size["io"], self.layer_size["cf"])
        self.cf2io = nn.Linear(self.layer_size["cf"], self.layer_size["io"])
        self.cf2cs = nn.Linear(self.layer_size["cf"], self.layer_size["cs"])
        self.cf2cf = nn.Linear(self.layer_size["cf"], self.layer_size["cf"])
        self.cs2cf = nn.Linear(self.layer_size["cs"], self.layer_size["cf"])
        self.cs2cs = nn.Linear(self.layer_size["cs"], self.layer_size["cs"])
        self.activate = activate

    def init_state(self, batch_size):
        device = next(self.parameters()).device
        self.last_output = None
        self.io_state = torch.zeros(size=(batch_size, self.layer_size["io"])).to(device)
        self.cf_state = torch.zeros(size=(batch_size, self.layer_size["cf"])).to(device)
        self.cs_state = torch.zeros(size=(batch_size, self.layer_size["cs"])).to(device)

        # fill_value = 0
        # self.last_output = torch.full(
        #     size=(batch_size, self.layer_size["out"]), fill_value=fill_value,
        # )
        # self.io_state = torch.full(
        #     size=(batch_size, self.layer_size["io"]), fill_value=fill_value
        # )
        # self.cf_state = torch.full(
        #     size=(batch_size, self.layer_size["cf"]), fill_value=fill_value
        # )
        # self.cs_state = torch.full(
        #     size=(batch_size, self.layer_size["cs"]), fill_value=fill_value
        # )

    # def _next_state(self, previous, new, tau):
    #     connected = torch.stack(new)
    #     new_summed = connected.sum(dim=0)
    #     ret = (1 - 1.0 / tau) * previous + new_summed / tau

    #     return self.activate(ret)

    def forward(self, x):  # x.shape(batch,x)
        if (
            self.last_output is not None and self.open_rate != 1
        ):  # start val is not changed by open_rate
            mix = x[:, : self.layer_size["out"]] * self.open_rate + self.last_output * (
                1 - self.open_rate
            )
            no_mix = x[:, self.layer_size["out"] :]
            x = torch.cat([mix, no_mix], axis=1)

        tau = self.tau["tau_io"]
        temp = (
            self.io2io(self.io_state) + self.cf2io(self.cf_state) + self.i2io(x)
        ) / tau + (1.0 - 1.0 / tau) * self.io_state
        new_io_state = self.activate(temp)

        tau = self.tau["tau_cf"]
        temp = (
            self.cf2cf(self.cf_state)
            + self.cs2cf(self.cs_state)
            + self.io2cf(self.io_state)
        ) / tau + (1.0 - 1.0 / tau) * self.cf_state
        new_cf_state = self.activate(temp)

        tau = self.tau["tau_cs"]
        temp = (self.cs2cs(self.cs_state) + self.cf2cs(self.cf_state)) / tau + (
            1.0 - 1.0 / tau
        ) * self.cs_state
        new_cs_state = self.activate(temp)
        # new_io_state = self._next_state(
        #     previous=self.io_state,
        #     new=[
        #         self.io2io(self.io_state),
        #         self.cf2io(self.cf_state),
        #         self.i2io(closed_x),
        #     ],
        #     tau=self.tau["tau_io"],
        # )
        # new_cf_state = self._next_state(
        #     previous=self.cf_state,
        #     new=[
        #         self.cf2cf(self.cf_state),
        #         self.cs2cf(self.cs_state),
        #         self.io2cf(self.io_state),
        #     ],
        #     tau=self.tau["tau_cf"],
        # )
        # new_cs_state = self._next_state(
        #     previous=self.cs_state,
        #     new=[
        #         self.cs2cs(self.cs_state),
        #         self.cf2cs(self.cf_state),
        #     ],
        #     tau=self.tau["tau_cs"],
        # )
        self.io_state = new_io_state
        self.cf_state = new_cf_state
        self.cs_state = new_cs_state
        y = self.activate(self.io2o(self.io_state))
        self.last_output = y.detach()
        return y
