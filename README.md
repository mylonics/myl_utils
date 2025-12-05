# myl_utils

A lightweight C++ utility library for embedded systems, designed for Zephyr RTOS.

## Extended Kalman Filter (EKF)

The EKF module provides sensor fusion for navigation applications, combining GPS and IMU data with optional magnetometer support.

### Features

- **9-state estimation**: Position (NED), Velocity (NED), and Orientation (Euler angles)
- **Lightweight**: Minimal memory footprint, suitable for embedded systems
- **GPS integration**: Position, velocity, and heading updates
- **IMU integration**: Gyroscope and accelerometer for high-rate prediction
- **Optional magnetometer**: Enable/disable via configuration
- **Configurable noise parameters**: Easy tuning for different platforms

### Quick Start

```cpp
#include <myl_utils/ekf.h>

using namespace myl_utils;

// Create EKF with default configuration
Ekf ekf;

// Or with custom configuration
EkfConfig config;
config.r_gps_position = 2.5f;  // Low noise GPS
config.use_magnetometer = true;
Ekf ekf_custom(config);

// Main loop
void update_loop() {
    // High-rate IMU prediction (100-1000 Hz)
    ImuData imu;
    imu.gyro_x = /* gyro x rad/s */;
    imu.gyro_y = /* gyro y rad/s */;
    imu.gyro_z = /* gyro z rad/s */;
    imu.accel_x = /* accel x m/s² */;
    imu.accel_y = /* accel y m/s² */;
    imu.accel_z = /* accel z m/s² */;
    
    ekf.Predict(imu, dt);  // dt in seconds

    // GPS update (1-10 Hz)
    if (gps_data_available) {
        GpsData gps;
        gps.position_n = /* north position m */;
        gps.position_e = /* east position m */;
        gps.position_d = /* down position m */;
        gps.velocity_n = /* north velocity m/s */;
        gps.velocity_e = /* east velocity m/s */;
        gps.velocity_d = /* down velocity m/s */;
        gps.ground_speed = /* ground speed m/s */;
        gps.heading = /* heading rad */;
        gps.valid = true;
        
        ekf.UpdateGps(gps);
    }

    // Optional magnetometer update
    if (mag_data_available) {
        MagData mag;
        mag.heading = /* computed heading rad */;
        mag.valid = true;
        
        ekf.UpdateMagnetometer(mag);
    }

    // Get estimated state
    EkfState state = ekf.GetState();
    // Use state.position_n, state.velocity_n, state.yaw, etc.
}
```

### Tuning Guide

#### Process Noise (Q matrix)

Controls trust in the prediction model vs measurements:
- **Higher Q values**: Less trust in prediction, more responsive to measurements
- **Lower Q values**: More trust in prediction, smoother but slower response

| Parameter | Range | Description |
|-----------|-------|-------------|
| `q_position` | 0.1 - 1.0 | Position process noise (m²) |
| `q_velocity` | 0.5 - 2.0 | Velocity process noise (m²/s²) |
| `q_orientation` | 0.01 - 0.1 | Orientation process noise (rad²) |

#### Measurement Noise (R matrix)

Set based on sensor specifications:

| Parameter | Range | Description |
|-----------|-------|-------------|
| `r_gps_position` | 2.5 - 10.0 | GPS position noise (m²), depends on fix quality |
| `r_gps_velocity` | 0.1 - 0.5 | GPS velocity noise (m²/s²) |
| `r_gps_heading` | 0.1 - 0.5 | GPS heading noise (rad²), unreliable at low speed |
| `r_mag_heading` | 0.1 - 0.3 | Magnetometer heading noise (rad²) |

#### Additional Settings

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_speed_for_heading` | 2.0 | Minimum speed (m/s) for GPS heading updates |
| `use_magnetometer` | false | Enable magnetometer fusion |

#### Tuning Tips

1. **Start with defaults**: Observe filter behavior before adjusting
2. **Slow response**: Increase Q values
3. **Noisy output**: Decrease Q or increase R values
4. **GPS heading at low speed**: GPS heading is derived from velocity and unreliable when stationary
5. **Magnetometer indoors**: Use with caution due to magnetic interference
6. **High dynamics**: Increase Q values for aggressive maneuvers
7. **RTK GPS**: Use lower `r_gps_position` (0.1 - 1.0) for high-precision GPS

### Coordinate Frames

- **NED (North-East-Down)**: Right-handed coordinate system
  - North: positive X direction
  - East: positive Y direction
  - Down: positive Z direction
- **Euler angles**: Roll-Pitch-Yaw (ZYX rotation order)
  - Roll: rotation about X axis
  - Pitch: rotation about Y axis
  - Yaw: rotation about Z axis (0 = North, positive = clockwise)

### Memory Usage

- State vector: 36 bytes (9 × float)
- Covariance matrix: 324 bytes (9 × 9 × float)
- Configuration: ~40 bytes
- **Total**: ~400 bytes RAM

### License

MIT License
