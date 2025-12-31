# Visualizer Component

## Requirements

- Python 3.10

## Runtime Preparation

Make sure to put mano model resources under `assets/mano`:

```text
./assets/mano
├── LICENSE.txt
├── __init__.py
├── models
├── ...
```

Execute these command in sequence to create a python environment with conda, then install the required packages:

```shell
conda create -n rfmocap python=3.10
conda activate rfmocap
pip install -r requirements.txt
```

On some linux systems, you might encounter `libdl.so` missing problem, causing RFU fail to load URDF. In this case, you should either

1. Install `libdl.so`:
   
   ```shell
   sudo apt-get install libc6-dev
   ```

2. Copy a `libdl.so` to your working directory.

3. Follow [this post](https://github.com/robotflow-initiative/rfuniverse/issues/3)
  
  ```shell
  sudo ln -s /lib/x86_64-linux-gnu/libdl.so.2 /lib/x86_64-linux-gnu/libdl.so
  sudo apt install minizip
  ```

## Run the visualizer

```shell
python main_gui.py
```

## Configuration Explained

The `main_gui.py` will automatically apply settings from `$WORKING_DIRECTORY/config.json`. The file looks like:

```json lines
{
    "grpc_address": "127.0.0.1:18890", // hand's remote grpc address
    "use_rfu": false, // use rfuniverse as visualization
    "use_cuda": true, // use cuda acceleration
    "param_base_quaterion": null, // calibration params
    "param_P": null,  //  calibration params
    "param_rot_bias": null,  // calibration params
    "param_rot_amp": null,  //  calibration params
    "param_mano_shape_params": null,  //  calibration params
    "data_store": "./",  // save data path
    "tracker_uid": null  // wrist tracker id
}
```

## Developer's Guide

### RaspberryPi's Date

`ntpd` will not sync time if there is a large time difference between the local time and the server time (30 minutes). To solve this, you can manually set the time:

```shell
sudo ntpdate <ntp_server>
```

### NOKOV SDK

Run this command to install nokov python SDK

```shell
pip install .\third_party\nokov\nokovpy-3.0.1-py3-none-any.whl 
```

### RFUniverse

Tele-operate demo requires RFUniverse binary, which need to be downloaded via this convenient command.

```shell
pyrfuniverse download
```

### OptiTrack Bridge

There was this [optitrack_bridge](https://github.com/davidliyutong/optitrack_bridge) project that proxies optitrack as gRPC server.


### FPC Connector

The fpc connectors are arranged as follows

- Left Hand
    ```text
    ---------------------
       palm  |  middle
    ---------------------
      index  |  ring
    ---------------------
      thumb  |  little
    ---------------------
    ```

- Right Hand
  ```text
    ---------------------
       palm  |  middle
    ---------------------
       ring  |  index
    ---------------------
     little  |  thumb
    ---------------------
    ```

MANO index

```
#   transform order of right hand
#         15-14-13-\
#                   \
#*   3-- 2 -- 1 -----0   < NOTE: demo on this finger
#   6 -- 5 -- 4 ----/
#   12 - 11 - 10 --/
#    9-- 8 -- 7 --/

#  the ID: 1 joints have been rotated by pi/6 around spread-axis, and pi/2 around bend-axis
#  the ID: 2, 3 joints have been rotated by pi/2 around bend-axis
```

## Calibration Pose

WARNING: vertical_up and vertical_down should have at least 90 degree of rotation difference.
