#pragma once

/**
 * @file ekf.h
 * @brief Lightweight Extended Kalman Filter for sensor fusion
 *
 * This EKF fuses GPS (position, velocity, heading) with IMU data (gyroscope,
 * accelerometer) and optionally magnetometer data for robust state estimation.
 *
 * ## State Vector (9 states):
 * - Position: x, y, z (meters, NED frame)
 * - Velocity: vx, vy, vz (m/s, NED frame)
 * - Orientation: roll, pitch, yaw (radians)
 *
 * ## Sensor Inputs:
 * - GPS: position (lat/lon/alt converted to NED), ground speed, heading
 * - IMU: gyroscope (rad/s), accelerometer (m/s²)
 * - Magnetometer (optional): heading reference
 *
 * ## Tuning Guide:
 *
 * ### Process Noise (Q matrix):
 * Controls how much the filter trusts the prediction model vs measurements.
 * - Higher values = less trust in prediction, more responsive to measurements
 * - Lower values = more trust in prediction, smoother but slower response
 *
 * Recommended starting values:
 * - Position: 0.1 - 1.0 (depends on vehicle dynamics)
 * - Velocity: 0.5 - 2.0 (higher for aggressive maneuvers)
 * - Orientation: 0.01 - 0.1 (lower for stable platforms)
 *
 * ### Measurement Noise (R matrix):
 * Set based on sensor specifications and observed noise.
 * - GPS position: typically 2.5 - 10.0 m (depends on fix quality)
 * - GPS velocity: typically 0.1 - 0.5 m/s
 * - GPS heading: typically 0.1 - 0.5 rad (depends on speed)
 * - Magnetometer: typically 0.1 - 0.3 rad
 *
 * ### Tips:
 * 1. Start with default values and observe filter behavior
 * 2. If filter is too slow to respond, increase Q values
 * 3. If filter is too noisy, decrease Q or increase R values
 * 4. GPS heading is unreliable at low speeds - consider disabling below 2 m/s
 * 5. Magnetometer can be affected by magnetic interference - use sparingly indoors
 */

#include <cmath>
#include <cstring>

namespace myl_utils {

/**
 * @brief Configuration parameters for the EKF
 */
struct EkfConfig {
  // Process noise (Q matrix diagonal)
  float q_position{0.5f};      ///< Position process noise (m²)
  float q_velocity{1.0f};      ///< Velocity process noise (m²/s²)
  float q_orientation{0.05f};  ///< Orientation process noise (rad²)

  // GPS measurement noise (R matrix diagonal)
  float r_gps_position{5.0f};   ///< GPS position noise (m²)
  float r_gps_velocity{0.3f};   ///< GPS velocity noise (m²/s²)
  float r_gps_heading{0.2f};    ///< GPS heading noise (rad²)

  // Magnetometer measurement noise
  float r_mag_heading{0.15f};   ///< Magnetometer heading noise (rad²)

  // Minimum speed for GPS heading update (m/s)
  float min_speed_for_heading{2.0f};

  // Enable/disable magnetometer fusion
  bool use_magnetometer{false};
};

/**
 * @brief IMU measurement data
 */
struct ImuData {
  float gyro_x{0.0f};   ///< Gyroscope x (rad/s)
  float gyro_y{0.0f};   ///< Gyroscope y (rad/s)
  float gyro_z{0.0f};   ///< Gyroscope z (rad/s)
  float accel_x{0.0f};  ///< Accelerometer x (m/s²)
  float accel_y{0.0f};  ///< Accelerometer y (m/s²)
  float accel_z{0.0f};  ///< Accelerometer z (m/s²)
};

/**
 * @brief GPS measurement data
 */
struct GpsData {
  float position_n{0.0f};     ///< North position (m) relative to origin
  float position_e{0.0f};     ///< East position (m) relative to origin
  float position_d{0.0f};     ///< Down position (m) relative to origin
  float velocity_n{0.0f};     ///< North velocity (m/s)
  float velocity_e{0.0f};     ///< East velocity (m/s)
  float velocity_d{0.0f};     ///< Down velocity (m/s)
  float ground_speed{0.0f};   ///< Ground speed (m/s)
  float heading{0.0f};        ///< Heading (rad), 0 = North, positive = clockwise
  bool valid{false};          ///< Data validity flag
};

/**
 * @brief Magnetometer measurement data
 */
struct MagData {
  float heading{0.0f};  ///< Computed heading from magnetometer (rad)
  bool valid{false};    ///< Data validity flag
};

/**
 * @brief EKF estimated state output
 */
struct EkfState {
  // Position (NED frame, meters)
  float position_n{0.0f};
  float position_e{0.0f};
  float position_d{0.0f};

  // Velocity (NED frame, m/s)
  float velocity_n{0.0f};
  float velocity_e{0.0f};
  float velocity_d{0.0f};

  // Orientation (Euler angles, radians)
  float roll{0.0f};   ///< Roll angle (-π to π)
  float pitch{0.0f};  ///< Pitch angle (-π/2 to π/2)
  float yaw{0.0f};    ///< Yaw/heading (0 to 2π)
};

/**
 * @brief Lightweight Extended Kalman Filter for sensor fusion
 *
 * This class implements a 9-state EKF for fusing GPS and IMU data.
 * The filter uses a simplified model suitable for ground vehicles and
 * low-dynamics aerial platforms.
 */
class Ekf {
 public:
  static constexpr int kNumStates = 9;

  /**
   * @brief Construct a new Ekf object with default configuration
   */
  Ekf() { Reset(); }

  /**
   * @brief Construct a new Ekf object with custom configuration
   * @param config EKF configuration parameters
   */
  explicit Ekf(const EkfConfig& config) : config_(config) { Reset(); }

  /**
   * @brief Reset the filter to initial state
   */
  void Reset() {
    // Reset state vector
    std::memset(state_, 0, sizeof(state_));

    // Initialize covariance matrix as diagonal
    std::memset(P_, 0, sizeof(P_));
    for (int i = 0; i < 3; ++i) {
      P_[i][i] = 100.0f;      // Position uncertainty
      P_[i + 3][i + 3] = 10.0f;  // Velocity uncertainty
      P_[i + 6][i + 6] = 1.0f;   // Orientation uncertainty
    }

    initialized_ = false;
  }

  /**
   * @brief Update configuration parameters
   * @param config New configuration
   */
  void SetConfig(const EkfConfig& config) { config_ = config; }

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const EkfConfig& GetConfig() const { return config_; }

  /**
   * @brief Prediction step using IMU data
   *
   * Call this at the IMU update rate (typically 100-1000 Hz).
   *
   * @param imu IMU measurement data
   * @param dt Time step in seconds
   */
  void Predict(const ImuData& imu, float dt) {
    if (dt <= 0.0f) return;

    // Extract current orientation
    float roll = state_[6];
    float pitch = state_[7];
    float yaw = state_[8];

    // Precompute trig functions
    float sr = std::sin(roll);
    float cr = std::cos(roll);
    float sp = std::sin(pitch);
    float cp = std::cos(pitch);

    // Protect against gimbal lock (pitch approaching ±π/2)
    // Clamp cos(pitch) to avoid division by zero
    const float kMinCosPitch = 0.001f;
    if (std::fabs(cp) < kMinCosPitch) {
      cp = (cp >= 0.0f) ? kMinCosPitch : -kMinCosPitch;
    }
    float tp = sp / cp;  // tan(pitch) = sin(pitch) / cos(pitch)

    // Update orientation using gyroscope
    // Convert body rates to Euler angle rates
    float roll_rate = imu.gyro_x + sr * tp * imu.gyro_y + cr * tp * imu.gyro_z;
    float pitch_rate = cr * imu.gyro_y - sr * imu.gyro_z;
    float yaw_rate = (sr / cp) * imu.gyro_y + (cr / cp) * imu.gyro_z;

    // Integrate orientation
    state_[6] += roll_rate * dt;
    state_[7] += pitch_rate * dt;
    state_[8] += yaw_rate * dt;

    // Normalize angles
    NormalizeAngles();

    // Rotation matrix from body to NED frame
    float sy = std::sin(state_[8]);
    float cy = std::cos(state_[8]);
    sp = std::sin(state_[7]);
    cp = std::cos(state_[7]);
    sr = std::sin(state_[6]);
    cr = std::cos(state_[6]);

    // Transform accelerometer to NED frame and remove gravity
    float accel_n = (cp * cy) * imu.accel_x +
                    (sr * sp * cy - cr * sy) * imu.accel_y +
                    (cr * sp * cy + sr * sy) * imu.accel_z;
    float accel_e = (cp * sy) * imu.accel_x +
                    (sr * sp * sy + cr * cy) * imu.accel_y +
                    (cr * sp * sy - sr * cy) * imu.accel_z;
    float accel_d = (-sp) * imu.accel_x +
                    (sr * cp) * imu.accel_y +
                    (cr * cp) * imu.accel_z + kGravity;

    // Update velocity
    state_[3] += accel_n * dt;
    state_[4] += accel_e * dt;
    state_[5] += accel_d * dt;

    // Update position
    state_[0] += state_[3] * dt + 0.5f * accel_n * dt * dt;
    state_[1] += state_[4] * dt + 0.5f * accel_e * dt * dt;
    state_[2] += state_[5] * dt + 0.5f * accel_d * dt * dt;

    // Update covariance: P = F*P*F' + Q
    // Using simplified diagonal Q for efficiency
    UpdateCovariancePrediction(dt);
  }

  /**
   * @brief Update step using GPS data
   *
   * Call this when new GPS data is available (typically 1-10 Hz).
   *
   * @param gps GPS measurement data
   */
  void UpdateGps(const GpsData& gps) {
    if (!gps.valid) return;

    // Position update (3 measurements mapping directly to states 0, 1, 2)
    float z_pos[3] = {gps.position_n, gps.position_e, gps.position_d};
    float R_pos[3] = {config_.r_gps_position, config_.r_gps_position,
                      config_.r_gps_position};
    ApplyMeasurementUpdate(z_pos, R_pos, 3, 0);

    // Velocity update (3 measurements mapping directly to states 3, 4, 5)
    float z_vel[3] = {gps.velocity_n, gps.velocity_e, gps.velocity_d};
    float R_vel[3] = {config_.r_gps_velocity, config_.r_gps_velocity,
                      config_.r_gps_velocity};
    ApplyMeasurementUpdate(z_vel, R_vel, 3, 3);

    // Heading update (only if moving fast enough)
    if (gps.ground_speed >= config_.min_speed_for_heading) {
      float z_hdg[1] = {gps.heading};
      float R_hdg[1] = {config_.r_gps_heading};
      ApplyMeasurementUpdateAngle(z_hdg, R_hdg, 1, 8);
    }

    initialized_ = true;
  }

  /**
   * @brief Update step using magnetometer data
   *
   * Call this when new magnetometer data is available.
   * Only used if use_magnetometer is enabled in config.
   *
   * @param mag Magnetometer measurement data
   */
  void UpdateMagnetometer(const MagData& mag) {
    if (!config_.use_magnetometer || !mag.valid) return;

    float z_hdg[1] = {mag.heading};
    float R_hdg[1] = {config_.r_mag_heading};
    ApplyMeasurementUpdateAngle(z_hdg, R_hdg, 1, 8);
  }

  /**
   * @brief Get the current estimated state
   * @return Current state estimate
   */
  EkfState GetState() const {
    EkfState result;
    result.position_n = state_[0];
    result.position_e = state_[1];
    result.position_d = state_[2];
    result.velocity_n = state_[3];
    result.velocity_e = state_[4];
    result.velocity_d = state_[5];
    result.roll = state_[6];
    result.pitch = state_[7];
    result.yaw = state_[8];
    return result;
  }

  /**
   * @brief Check if the filter has been initialized with GPS data
   * @return true if initialized
   */
  bool IsInitialized() const { return initialized_; }

  /**
   * @brief Get position uncertainty (standard deviation in meters)
   * @return Position uncertainty estimate
   */
  float GetPositionUncertainty() const {
    return std::sqrt(P_[0][0] + P_[1][1] + P_[2][2]);
  }

  /**
   * @brief Get velocity uncertainty (standard deviation in m/s)
   * @return Velocity uncertainty estimate
   */
  float GetVelocityUncertainty() const {
    return std::sqrt(P_[3][3] + P_[4][4] + P_[5][5]);
  }

  /**
   * @brief Get heading uncertainty (standard deviation in radians)
   * @return Heading uncertainty estimate
   */
  float GetHeadingUncertainty() const {
    return std::sqrt(P_[8][8]);
  }

 private:
  static constexpr float kGravity = 9.81f;
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kTwoPi = 2.0f * kPi;

  EkfConfig config_;
  float state_[kNumStates];
  float P_[kNumStates][kNumStates];
  bool initialized_{false};

  /**
   * @brief Normalize angles to proper ranges
   */
  void NormalizeAngles() {
    // Roll: -π to π
    while (state_[6] > kPi) state_[6] -= kTwoPi;
    while (state_[6] < -kPi) state_[6] += kTwoPi;

    // Pitch: -π/2 to π/2 (clamped, not wrapped)
    if (state_[7] > kPi / 2.0f) state_[7] = kPi / 2.0f;
    if (state_[7] < -kPi / 2.0f) state_[7] = -kPi / 2.0f;

    // Yaw: 0 to 2π
    while (state_[8] >= kTwoPi) state_[8] -= kTwoPi;
    while (state_[8] < 0.0f) state_[8] += kTwoPi;
  }

  /**
   * @brief Normalize an angle difference for proper wrapping
   */
  static float NormalizeAngleDiff(float diff) {
    while (diff > kPi) diff -= kTwoPi;
    while (diff < -kPi) diff += kTwoPi;
    return diff;
  }

  /**
   * @brief Update covariance matrix during prediction step
   */
  void UpdateCovariancePrediction(float dt) {
    // Simplified covariance update: P = P + Q*dt
    // This is a conservative approximation that grows uncertainty over time
    float dt2 = dt * dt;

    for (int i = 0; i < 3; ++i) {
      P_[i][i] += config_.q_position * dt2;
      P_[i + 3][i + 3] += config_.q_velocity * dt2;
      P_[i + 6][i + 6] += config_.q_orientation * dt2;
    }

    // Cross-correlation between position and velocity
    for (int i = 0; i < 3; ++i) {
      P_[i][i + 3] += P_[i + 3][i + 3] * dt;
      P_[i + 3][i] = P_[i][i + 3];
    }

    // Limit covariance growth to prevent numerical issues
    for (int i = 0; i < kNumStates; ++i) {
      if (P_[i][i] > 1000.0f) P_[i][i] = 1000.0f;
    }
  }

  /**
   * @brief Apply Kalman measurement update for linear measurements
   *
   * Uses a simplified sequential update where each measurement maps directly
   * to one state (identity H matrix for the specific state indices).
   *
   * @param z Measurement values
   * @param R Measurement noise variances
   * @param num_meas Number of measurements
   * @param state_offset Starting state index for measurements
   */
  void ApplyMeasurementUpdate(const float* z, const float* R,
                              int num_meas, int state_offset) {
    for (int m = 0; m < num_meas; ++m) {
      int idx = state_offset + m;

      // Innovation: y = z - H*x
      float y = z[m] - state_[idx];

      // Innovation covariance: S = H*P*H' + R = P[idx][idx] + R[m]
      float S = P_[idx][idx] + R[m];

      if (S < 1e-6f) continue;  // Avoid division by zero

      // Kalman gain (simplified for single measurement)
      float K[kNumStates];
      for (int i = 0; i < kNumStates; ++i) {
        K[i] = P_[i][idx] / S;
      }

      // State update: x = x + K*y
      for (int i = 0; i < kNumStates; ++i) {
        state_[i] += K[i] * y;
      }

      // Covariance update: P = (I - K*H)*P
      // Simplified: P[i][j] -= K[i] * P[idx][j] for all i,j
      for (int i = 0; i < kNumStates; ++i) {
        for (int j = 0; j < kNumStates; ++j) {
          P_[i][j] -= K[i] * P_[idx][j];
        }
      }
    }

    // Ensure symmetry and positive definiteness
    for (int i = 0; i < kNumStates; ++i) {
      for (int j = i + 1; j < kNumStates; ++j) {
        float avg = 0.5f * (P_[i][j] + P_[j][i]);
        P_[i][j] = avg;
        P_[j][i] = avg;
      }
      if (P_[i][i] < 0.0001f) P_[i][i] = 0.0001f;
    }
  }

  /**
   * @brief Apply Kalman measurement update for angle measurements
   *
   * Handles angle wrapping for heading updates. Uses a simplified sequential
   * update where each measurement maps directly to one state.
   *
   * @param z Measurement values (angles in radians)
   * @param R Measurement noise variances
   * @param num_meas Number of measurements
   * @param state_offset Starting state index for measurements
   */
  void ApplyMeasurementUpdateAngle(const float* z, const float* R,
                                   int num_meas, int state_offset) {
    for (int m = 0; m < num_meas; ++m) {
      int idx = state_offset + m;

      // Innovation with angle wrapping
      float y = NormalizeAngleDiff(z[m] - state_[idx]);

      // Innovation covariance
      float S = P_[idx][idx] + R[m];

      if (S < 1e-6f) continue;

      // Kalman gain
      float K[kNumStates];
      for (int i = 0; i < kNumStates; ++i) {
        K[i] = P_[i][idx] / S;
      }

      // State update
      for (int i = 0; i < kNumStates; ++i) {
        state_[i] += K[i] * y;
      }

      // Covariance update
      for (int i = 0; i < kNumStates; ++i) {
        for (int j = 0; j < kNumStates; ++j) {
          P_[i][j] -= K[i] * P_[idx][j];
        }
      }
    }

    // Normalize angles after update
    NormalizeAngles();

    // Ensure symmetry and positive definiteness
    for (int i = 0; i < kNumStates; ++i) {
      for (int j = i + 1; j < kNumStates; ++j) {
        float avg = 0.5f * (P_[i][j] + P_[j][i]);
        P_[i][j] = avg;
        P_[j][i] = avg;
      }
      if (P_[i][i] < 0.0001f) P_[i][i] = 0.0001f;
    }
  }
};

}  // namespace myl_utils
