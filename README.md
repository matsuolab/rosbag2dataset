# rosbag2dataset

## How to Use
```
$ cd docker
$ ./BUILD_DOCKER_IMAGE.sh
$ ./RUN_DOCKER_CONTAINER.sh BAGFILE_DIR DATASET_DIR
```
In the container,
```
$ python3 rosbag2dataset.py
```

## Config
If `bagfile_names` is empty, convert all of the bagfiles in the source directory.
