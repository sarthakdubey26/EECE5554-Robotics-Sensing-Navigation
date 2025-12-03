# Lab 5 - Vehicle IMU/GPS Navigation

Sensor fusion lab using VectorNav VN-100 and GPS for vehicle navigation.

## Setup

**Hardware:**
- VectorNav VN-100 IMU
- GPS receiver (same from lab 1)

**Data Collection** (Nov 21, 2023):
Team collected two datasets using Rafael's car:
- `data_going_in_circles.bag` - calibration run at Ruggles Circle
- `data_driving.bag` - ~10 min drive around Boston, returned to start

## Running Analysis

Requirements: `numpy`, `scipy`, `pandas`, `matplotlib`, `rosbag2`
```bash
# Extract data from bags first
python3 analysis/extract_rosbag.py  # if you made one

# Run calibration
python3 analysis/mag_calibration.py

# Complementary filter
python3 analysis/comp_filter.py

# Velocity and trajectory
python3 analysis/velocity_calc.py
python3 analysis/trajectory.py
```

## Results

Magnetometer calibration cleaned up heading by ~25-30 degrees. Complementary filter (0.05 Hz cutoff) worked pretty well, matched VectorNav output closely. 

IMU trajectory stayed accurate for about 60 seconds before drift took over. GPS velocity helped validate the accel integration but there was still ~15% systematic underestimation.

## Files

- `launch/` - launch file for GPS+IMU
- `analysis/` - processing scripts
- `plots/` - generated figures for report
- `data/` - rosbag files (too large for git, available on request)

## Notes

- Had to apply 0.85x scaling factor to IMU trajectory to match GPS distances
- Gravity removal was critical for velocity estimation
- Position error grows quadratically, not viable for >1-2 min without GPS updates
