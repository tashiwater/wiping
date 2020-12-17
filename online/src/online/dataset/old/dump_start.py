#!/usr/bin/env python
# coding: utf-8
import os
from pathlib import Path


if __name__ == "__main__":
    step = 20

    yaml_line_num = 25+33*step - 7
    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = CURRENT_DIR + "/../../../data/"
    motion_data_dir = DATA_DIR + "1116/"
    YAML_DIR = motion_data_dir + "/direct/"
    output_dir = motion_data_dir + "/start_motion/"
    paths = [str(p) for p in Path(YAML_DIR).glob("./*_raw.yaml")]
    paths.sort()
    for i, path in enumerate(paths):
        print(path)
        with open(path, "r") as f:
            datalist = f.readlines()
            output_path = output_dir + "{}.yaml".format(i)
            with open(output_path, "w") as fw:
                fw.writelines(datalist[:yaml_line_num])

                fw.writelines(["      - 0.0\r\n" for _ in range(7)])
