#!/bin/bash
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'n'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'x'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'p'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'lowpower'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'n'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'x'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'p'}"
sleep .5
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'stop'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'hazard'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'sadface'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'reverse'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'forwardgreen'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'rightgreen'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'backwardgreen'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'leftgreen'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'forwardyellow'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'leftyellow'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'backwardyellow'}"
sleep 1
ros2 topic pub -1 /led_pattern std_msgs/msg/String "{data: 'rightyellow'}"
sleep 1
