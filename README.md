# rosbag2dataset

## How to Use
```
$ ./RUN_DOCKER_CONTAINER.sh path/to/bagfiles path/to/dataset
```

## Example
### Run Docker container
```
./RUN_DOCKER_CONTAINER.sh /robot-qnap-1/teleop_dataset/reacher_demo /robot-qnap-1/bc_dataset/reacher
./RUN_DOCKER_CONTAINER.sh teleop dataset
```
### Run rosbag2dataset.py to convert rosbag to torch tensor dataset
```
python3 rosbag2dataset.py
```

## Debag rosbag
```
pip install scipy matplotlib pytransform3d
python3 rosbag_cat.py
```