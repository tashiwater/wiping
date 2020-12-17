#!/usr/bin/env python
# coding: utf-8

from manager.manager_base import ManagerBase
from model.MTRNN_cs import MTRNN
import torch


class Manager(ManagerBase):
    def set_MTRNN(self):
        in_size, out_size = 30, 30
        self._net = MTRNN(
            36,
            layer_size={"in": in_size, "out": out_size,
                        "io": 50, "cf": self._cf_num, "cs": self._cs_num},
            tau={"tau_io": 2, "tau_cf": 10, "tau_cs": 30},
            open_rate=self._open_rate,
            activate=torch.nn.Tanh()
        )
        model_path = self._model_dir + \
            "MTRNN/1127/noimg/5000/{}_{}.pth".format(
                self._cf_num, self._cs_num)
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))
        self._net.load_state_dict(checkpoint["model"])
        print(self._net)
        self._net.eval()
        each_container = 3
        num = self._container*each_container
        self._net.init_state(1, self._net.cs0[num:num+1])


if __name__ == "__main__":
    before_scale = [[-0.2547831567492811, 0.3522597794941982],
                    [-0.19786794214487075, 0.5512273242084056],
                    [-0.47539278851585526, 0.23994786809418345],
                    [-0.6276378933223615, 0.4227885409588754],
                    [-0.1361531453344059, 0.6384239953428051],
                    [-0.9111142056498871, 0.2545039097448242],
                    [-0.29218565636279004, 0.6055594556477641],
                    [-14.483999252319336, 24.43199920654297],
                    [-20.572500705718994, 11.095499992370605],
                    [-6.2639994621276855, 13.763999283313751],
                    [-13.971332907676697, 14.211999297142029],
                    [-1.749666690826416, 2.956333249807358],
                    [-6.40000057220459, 3.5500000715255737],
                    [-2.929666757583618, 5.298333764076233], ]
    manager = Manager()
    manager.init(before_scale)
    manager.run()
