# Waterlinked DVL ROS Driver

This repository contains ROS Driver for Waterlinked DVL.

![DVL Image](https://waterlinkshop.wpenginepowered.com/wp-content/uploads/2021/12/wl-21035-3_dvl-a50_side4_1600.jpg)


The driver currently supports TCP communication.
The project is safety use yet the interface is subject to change.

## ROS Interface

### Topics

- Topic: `dvl/twist`

  Message Type: `geometry_msgs::TwistWithCovarianceStamped`

- Topic: `dvl/pose`

  Message Type: `geometry_msgs::PoseWithCovarianceStamped`

- Topic: `dvl/transducer_report`

  Message Type: `waterlinked_dvl::TransducerReportStamped`

- Topic: `dvl/position_report`

  Message Type: `waterlinked_dvl::PositionReportStamped`

### Parameters

- `ip`
- `frame_id`
- `velocity_covariance`
- `position_covariance`
- `speed_of_sound`
- `mouting_rotation_offset`
- `acoustics_enabled`

### Services

- `acoustics_enabled`
- `get_last_response`
- `get_running_config`