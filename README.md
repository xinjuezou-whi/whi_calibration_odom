# whi_calibration_odom
tool for calibrating the odometry

## Usage
1. run motion interface, taking L2 differential4 as an example:
   ```
   roslaunch whi_motion_interface NaviBOT.launch vehicle:=L2 vehicle_model:=differential4
   ```

2. run calibration node
   ```
   roslaunch whi_calibration_odom whi_calibration_odom.launch 
   ```
   
2. calibrating linear
   ```
   rosservice call /whi_calibration_odom/linear "{}" 
   ```
   
3. calibrating angular
   ```
   rosservice call /whi_calibration_odom/angular "{}" 
   ```
