# EECE 5554 Lab 4: Inertial Odometry

**Student:** Sarthak Dubey  
**Email:** dubey.sart@northeastern.edu  
**Date:** October 29, 2025

---

## Circle Walk Analysis

### Figure 1: Magnetometer Calibration (Circle)
![](plots/fig1_circle_mag_calibration.png)

### Figure 2: X-axis Rotation (Circle)
![](plots/fig2_circle_x_rotation.png)

### Figure 3: Y-axis Rotation (Circle)
![](plots/fig3_circle_y_rotation.png)

### Figure 4: Z-axis and Magnetometer Heading (Circle)
![](plots/fig4_circle_z_heading.png)

### Figure 5: X-axis Acceleration and Velocity (Circle)
![](plots/fig5_circle_x_accel_vel.png)

### Figure 6: Y-axis Acceleration and Velocity (Circle)
![](plots/fig6_circle_y_accel_vel.png)

### Figure 7: Z-axis Acceleration and Velocity (Circle)
![](plots/fig7_circle_z_accel_vel.png)

### Figure 8: Position Estimate (Circle)
![](plots/fig8_circle_position.png)

---

\newpage

## Square Walk Analysis

### Figure 9: Magnetometer Calibration (Square)
![](plots/fig9_square_mag_calibration.png)

### Figure 10: X-axis Rotation (Square)
![](plots/fig10_square_x_rotation.png)

### Figure 11: Y-axis Rotation (Square)
![](plots/fig11_square_y_rotation.png)

### Figure 12: Z-axis and Magnetometer Heading (Square)
![](plots/fig12_square_z_heading.png)

### Figure 13: X-axis Acceleration and Velocity (Square)
![](plots/fig13_square_x_accel_vel.png)

### Figure 14: Y-axis Acceleration and Velocity (Square)
![](plots/fig14_square_y_accel_vel.png)

### Figure 15: Z-axis Acceleration and Velocity (Square)
![](plots/fig15_square_z_accel_vel.png)

### Figure 16: Position Estimate (Square)
![](plots/fig16_square_position.png)

---

\newpage

## Discussion Questions

### Question 1: Observations and Accuracy

The position estimates from both magnetometer and gyro headings show significant drift and errors, which is expected for pure inertial navigation without external references. The circle walk path does not close into a perfect circle, and the square walk does not form clean 90-degree corners, demonstrating the challenges of double-integration of noisy accelerometer data and gyro drift accumulation over time. However, the general pattern of circular motion is visible in Figure 8, and the square path in Figure 16 shows approximate right-angle turns, indicating the IMU captured the overall trajectory structure despite quantitative errors of several meters.

### Question 2: Improvements for Lab 5

For Lab 5 vehicle data collection, I would ensure the IMU is rigidly mounted to the vehicle chassis with the X-axis precisely aligned with the vehicle's forward direction to minimize misalignment errors. I would also place the IMU away from magnetic interference sources like motors, batteries, and metal components, and verify the magnetometer calibration in the vehicle environment before data collection. Additionally, I would collect longer stationary periods at the beginning and end of each run to establish bias estimates and enable zero-velocity updates for drift correction.

