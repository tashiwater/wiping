# wiping (for motion generation)
## Requirements
- sudo pip install torch torchvision
- sudo pip install future
- sudo pip install pandas

## Run
You need to rewrite parameters and paths in manager_{your model}.py before you execute it 
- bash bash_chmod.sh # chmod tactile sensor
- roslaunch wiping_bringup wiping_bringup.launch # launch torobo & camera 
- rosrun touchence sensing.py # tactile sensor
- roslaunch torobo_gui torobo_whole_body_manager.launch # GUI

### Take teach data
- use GUI. push teach -> save. then outputs are in data/output

### Generate online motion
- rosrun online manager_{your model}.py {container_ID}

## GUI
### Record motion to start posi
1. Teaching Mode
1. Traj Run
1. move your hand
1. Record Stop
1. Save RosParam

### Go to start posi
1. Load RosParam
1. Traj Run
1. decide start posi
1. then the robot got to start posi if you push "go start posi" button

### Take data 
1. teach start
1. save -> dump noised motion yaml to data/output/direct
1. go to start posi
1. Load RosParam <- 2's data
1. replay
1. save
1. go to start posi



## Contents
 - online/data/log/ : output files
 - online/data/model/ : *.pth

