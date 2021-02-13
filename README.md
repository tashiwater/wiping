# wiping
## Requirements
- sudo pip install torch torchvision
- sudo pip install future
- sudo pip install pandas

## Run
You need to rewrite parameters and paths in manager_{your model}.py before you execute it 
- bash bash_chmod.sh
- roslaunch wiping_bringup wiping_bringup.launch 
- rosrun touchence sensing.py
- roslaunch torobo_gui torobo_whole_body_manager.launch

### Take teach data
- use GUI. push teach -> save. then outputs are in data/output

### Generate online motion
- rosrun online manager_{your model}.py {container_ID}

## Contents
 - online/data/log/ : output files
 - online/data/model/ : *.pth

