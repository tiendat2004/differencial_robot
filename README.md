1.Launch urdf
roslaunch robot_description display.launch

![Screenshot 2025-02-23 163009](https://github.com/user-attachments/assets/a4cacc40-c3cf-476f-b093-e1f6a260bbd6)
2.Slam with hector_mapping
roslaunch robot_description roslaunch robot_control ros_control.launch
![Screenshot 2024-12-19 191638](https://github.com/user-attachments/assets/18db48f3-c62d-4bf6-9ab7-5611fcc8332d)
2.1.Save map
rosrun map_server map_saver -f ~/map
#################
3.Launch navigation
roslaunch robot_navigation navigation.launch
![Screenshot 2024-12-19 212347](https://github.com/user-attachments/assets/d3df0ea0-0530-4db3-8771-954a88f1af37)
