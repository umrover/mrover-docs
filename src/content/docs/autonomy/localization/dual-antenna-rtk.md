---
title: "Dual Antenna RTK"
---
## Overview: What is RTK?
<img src="https://github.com/umrover/mrover-ros/assets/91025934/1915dd40-7a4b-4a04-84af-afb47a45d35a" alt="image" width="700"/>
<br>
Real time kinematic positioning (RTK) is a technique used for enhancing the accuracy of positional data from GPS's. Essentially, RTK works by comparing the phase of the carrier wave of the signals received from GPS satellites with a reference station that is known to have a precisely known location. This reference station is typically a fixed, known location with a high-precision GPS receiver. The difference between the phase of the carrier wave at the reference station and the phase observed by the mobile receiver is used to calculate very precise position information. By using this correction data from the reference station in real-time, the mobile receiver can achieve centimeter-level accuracy in its position determination.

Please watch these videos to gain a deeper understanding of [RTK](https://youtu.be/ieearzWTCZw?si=BYbE3sEbV8UHbsDW) and how [GPS's work](https://youtu.be/ldusExdni0A?si=Q0yavfrlGoGCw3eu).

## Orientation with Dual Antennas
![dual_antenna_orientation](https://github.com/user-attachments/assets/c0b6b51d-1578-4d3d-b22f-18ebfe8ca9cd)

Using 2 antennas on the rover allows us to gain further information on the rover's heading (and pitch, depending on the orientation of the antennas). This is an excellent source of orientation information as it is mathematically derived solely from the position of the 2 rover antennas in relation to each other, making it highly resistant to external factors that may affect other sensors. Obtaining heading information from a dual antenna RTK system is particularly desirable since the only other sensor that gives heading is the magnetometer, which becomes significantly less accurate in the presence of external magnetic fields (ex. an area with a high concentration of metals). 

Dual antenna RTK not only allows us to achieve highly accurate positioning information, but attitude measurements as well. It can be enabled on dual GNSS receivers such as the [simpleRTK3B Compass](https://www.ardusimple.com/product/simplertk3b-compass/). 

## Goals and steps

### Configure simpleRTK3B Compass and feed RTCM corrections 
The [simpleRTK3B Compass (UM982)](https://www.ardusimple.com/product/simplertk3b-compass/) dual‑antenna receiver provides <1 cm position and ≈0.14° heading (≈1 m baseline) when it receives RTCM3 corrections from a fixed basestation. In this codebase, `basestation_gps_driver.py` reads UBX/RTCM from the base receiver and publishes raw corrections on the `rtcm` ROS 2 topic, and the rover’s C++ `rover_gps_driver` subscribes to `rtcm` and writes those bytes directly to the UM982 over serial while parsing NMEA to publish `/gps/fix` and `/gps_fix_status`. To output heading, enable UNIHEADING on the UM982 (via UPrecise or commands), then (optionally) re‑enable the commented UNIHEADING handling in `localization/rover_gps_driver.cpp` to publish `/heading/fix` and `/heading_fix_status`. Verify corrections with `ros2 topic hz /rtcm` and `ros2 topic echo /rtcm -n 1`.

### Publishing heading from the rover driver

The rover GPS driver actively parses Unicore UNIHEADING messages and publishes RTK heading and its status.

- Message gate
  - The driver looks for `#UNIHEADINGA` in the incoming Unicore stream and parses its fields.
- Status mapping
  - `NARROW_INT` → `FIXED`
  - `NARROW_FLOAT` → `FLOAT`
  - Any other status → `NO_SOL` (logs a warning and publishes `heading=0` with `NO_SOL`)
- Publications
  - `/heading/fix` (`mrover/Heading`): heading in degrees from the UM982 dual‑antenna solution
  - `/heading_fix_status` (`mrover/FixStatus`): one of `NONE | FLOAT | FIXED | NO_SOL`
- Prerequisite on the receiver
  - Ensure the UM982 is configured to output UNIHEADING at your desired rate (via UPrecise). Without UNIHEADING output, the driver cannot publish heading topics.
- Quick verification
  - `ros2 topic echo /heading_fix_status -n 1` (expect `FLOAT` or `FIXED` when dual‑antenna heading is working)
  - `ros2 topic echo /heading/fix -n 1` (verify heading updates)
  - If you see `NO_SOL`, check dual‑antenna wiring (master/slave ports), baseline configuration, and UM982 heading output settings.


## Dual‑Antenna RTK Heading

### Concept and why dual antennas
Dual‑antenna RTK heading computes yaw (and optionally pitch/roll depending on antenna geometry) from the fixed baseline vector between two GNSS antennas on the rover. The receiver phase‑differences the two antenna signals to estimate the azimuth of the baseline, yielding sub‑degree, drift‑free heading even at low speed or standstill where IMU yaw is weakly observable. With the [ArduSimple simpleRTK3B Compass (Unicorecomm UM982)](https://www.ardusimple.com/product/simplertk3b-compass/), a ~1.0 m baseline typically achieves ≈0.14° heading accuracy (per product spec).

Key properties:
- Heading is geometric (from the two‑antenna phase baseline) and does not depend on magnetometers.
- RTK corrections are not strictly required to compute heading; however, our stack uses heading “status” (FixStatus) to gate usage:
  - `heading_filter`: uses RTK heading only when its status is FIXED (see code below).
  - `iekf_se3`: accepts heading whenever status is not NO_SOL (weights controlled by `rtk_heading_noise`).

## How the dual‑antenna heading code works (for new members)

### IEKF: using RTK heading as a measurement
The IEKF fuses dual‑antenna RTK heading as a yaw measurement. It gates out invalid data, converts the heading into a body‑frame reference vector, builds a measurement model, and runs the EKF correction step weighted by `rtk_heading_noise`.

```302:330:localization/iekf_se3/iekf_se3.cpp
void IEKF_SE3::rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &rtk_heading, const mrover::msg::FixStatus::ConstSharedPtr &rtk_heading_status) {

    if (rtk_heading_status->fix_type.fix == mrover::msg::FixType::NO_SOL) {
        return;
    }

    Matrix36d H;
    Vector4d Y;
    Vector4d b;
    Matrix33d N;

    Vector3d rtk_heading_ref{1, 0, 0};

    double heading = 90 - fmod(rtk_heading->heading + 90, 360);

    if (heading < -180) {
        heading = 360 + heading;
    }

    heading = heading * (M_PI / 180);

    H << -1 * manif::skew(rtk_heading_ref), Matrix33d::Zero();
    Y << -1 * Vector3d{std::cos(heading), -std::sin(heading), 0}, 0;
    b << -1 * rtk_heading_ref, 0;
    N << X.block<3, 3>(0, 0) * rtk_heading_noise * Matrix33d::Identity() * X.block<3, 3>(0, 0).transpose();

    correct(Y, b, N, H);

}
```

- **Gate invalid**: If `/heading_fix_status` is `NO_SOL`, ignore the measurement (avoid corrupting the state).
- **Normalize heading**: Convert degrees→radians and wrap to \(-\pi,\pi\). The 90° offsets align the receiver’s heading convention with the rover’s +X forward axis used in the filter.
- **Measurement model**:
  - `rtk_heading_ref = [1,0,0]` is the rover forward direction in body frame.
  - `H` maps a small SO(3) error to the expected change in the forward direction.
  - `Y` encodes the measured forward direction from the heading angle.
  - `b` is the nominal expected direction.
  - `N` is the measurement covariance; it’s the body‑frame rotation scaled by the parameter `rtk_heading_noise`.
- **Correct()**: Runs the EKF update with this yaw constraint. Lower `rtk_heading_noise` = trust heading more.

Tip: Tune `iekf_se3.rtk_heading_noise` in `config/localization.yaml` to balance heading vs IMU.

### Heading filter: correcting IMU yaw when RTK heading is FIXED
This node applies a 1D Kalman filter to correct yaw using dual‑antenna heading only when the heading solution is `FIXED` (highest quality). It computes the uncorrected yaw from the IMU quaternion, forms an angle error to the measured RTK heading, then predict–corrects the scalar filter.

```71:99:localization/heading_filter/heading_filter.cpp
void HeadingFilter::sync_rtk_heading_callback(const mrover::msg::Heading::ConstSharedPtr &heading, const mrover::msg::FixStatus::ConstSharedPtr &heading_status) {


    if (!last_imu) {
        RCLCPP_WARN(get_logger(), "No IMU data!");
        return;
    }

    Eigen::Quaterniond uncorrected_orientation(last_imu->orientation.w, last_imu->orientation.x, last_imu->orientation.y, last_imu->orientation.z);
    R2d uncorrected_forward = uncorrected_orientation.toRotationMatrix().col(0).head(2);
    double uncorrected_heading = std::atan2(uncorrected_forward.y(), uncorrected_forward.x());

    // correct with rtk heading only when heading is fixed
    if (heading_status->fix_type.fix == mrover::msg::FixType::FIXED) {
        double measured_heading = fmod(heading->heading + 90, 360);
        measured_heading = 90 - measured_heading;
        if (measured_heading < -180) {
            measured_heading = 360 + measured_heading;
        }
        measured_heading = measured_heading * (M_PI / 180);

        double heading_correction_delta = measured_heading - uncorrected_heading;
        heading_correction_delta = fmod((heading_correction_delta + 3 * M_PI), 2 * M_PI) - M_PI;
        predict(get_parameter("process_noise").as_double());
        correct(heading_correction_delta, get_parameter("rtk_heading_noise").as_double());
    }


}
```

- **IMU gate**: If there’s no recent IMU, skip.
- **Uncorrected yaw**: Extract yaw from the IMU quaternion’s forward axis.
- **Use only FIXED**: Trust RTK heading only at best quality.
- **Angle wrap & KF**: Compute wrapped yaw error, then `predict(process_noise)` and `correct(error, rtk_heading_noise)`. Parameters live in `heading_filter` section of `config/localization.yaml`.

Result: A stable yaw correction that’s only applied when RTK heading is rock‑solid.

## 2025-26 Projects (relating to Dual Antenna RTK)

### Reduce-EMI and Triple Band Antenna Testing 

Context: We had an issue where connecting the ZED usb-c cable to the ZED camera (on rover) would switch the gps fix status `gps_fix_status` from 2 (RTK Fix), to 0 (No RTK Fix). This means that the rover is not publishing centermeter-level accurate rover positional data. We assume this has something to do with the ZED producing EMI once it is connected to the PDB system of the rover. The ZED camera we use contains high-speed microprocessors and other circuitry that operate at high frequencies, and hence become a generator of RF noise that impact the GPS fix. 

Methods (Ideas that require research + implementation):
* Copper foiling the ZED Camera housing and USB-C cable and ground it to the chassis of the rover.
* Elevate the dual antenna system, so as to maximise separation between the u-blox antennas and the e-box (preliminary designs being created).
* Adding ferrite beads to the antenna's cable opening in the e-box.
* Switching out dual-band antennas to our new triple-band antennas (Antenna can recieve radio signals within three distinct range of frequencies). This means greater satellite strength -> increased EMI resilience -> faster RTK convergence time.
* U-Center MON-SPAN (Spectrum Analyser)
* Plotting GPS satellite strength and multi-path interference to verify internal or external EMI.

Desired outcome:
* Auton missions should operate with a consistent and reliable RTK fix (state 2).
* Connecting the ZED config to the rover should not increase GPS covariance and hence cause the GPS reciever to cause lock.
* Satellite strength should be ranging from 40-50 dBs rather than 30-40 dBs. 