---
title: "Localization"
---
# What is Localization?
In simple terms, localization is just figuring out where the robot is. In the case of our rover, we need to know what point it is located at in 3D space, as well as what angle it is rotated to in each axis. This information is also known as a pose (see [this](/autonomy/3d-poses-transforms-rotations) page). We need to have this data at all times, and it needs to be updated frequently enough to keep up with the rover's motion. We typically estimate this data by measuring it with a variety of sensors and then processing that sensor data with a variety of fusion and filtering algorithms to make it more accurate.

# Sensors
## GPS
GPS (Global Positioning System) is a system that uses signals from an array of satellites orbiting earth to figure out where you are on the planet (read more about how this works [here](https://en.wikipedia.org/wiki/Global_Positioning_System)). We use a GPS unit mounted to the rover, which outputs our position on the earth in degrees latitude and degrees longitude.

<p align="center">
   <img src="https://user-images.githubusercontent.com/32557768/189783892-0ada5db3-5be6-4ce7-8665-06aa7e897b7e.png" width="600">
</p>

### What is a GPS Driver?
<img align="right" src="https://user-images.githubusercontent.com/32557768/192074714-abb42cc7-c791-4b06-892a-503c4229689d.png" width="400" />

A GPS driver is the glue between a GPS receiver’s serial/binary output and the ROS 2 topics the rest of localization stack consumes. It:
- Opens the receiver’s USB or serial port.
- Parses messages coming out of the receiver (NMEA 0183 at minimum; some receivers also speak UBX or Unicore proprietary).
- Publishes standard ROS 2 messages for downstream nodes. On the rover it also forwards RTCM3 corrections into the receiver input so RTK can converge (We will cover the RTK technique in the following paragraphs).

We run two cooperating GPS drivers: one on the basestation to produce RTK corrections, and one on the rover to consume those corrections and publish the rover’s position and status.

#### Reference background

- NMEA spec overview: [NMEA 0183](https://en.wikipedia.org/wiki/NMEA_0183)
  - A simple, ASCII text protocol emitted by GNSS receivers. Each “sentence” (e.g., GGA, RMC, GSA, GSV, GST) carries a specific set of fields such as time, latitude/longitude, fix type, satellite list, and estimated errors.
  - Drivers read these sentences line-by-line from a serial port (or TCP) and extract geodetic position and quality indicators for use by the rest of the stack.
- Common ROS package for NMEA → NavSatFix: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver)
  - Fortunately, a ROS package that listens to a serial/TCP NMEA stream and converts it into standard ROS messages, primarily `sensor_msgs/NavSatFix` on a `/fix` topic.
  - It handles timestamping, `frame_id`, and (when available) pulls covariance from GST. It does not generate RTK corrections; it just translates the receiver’s NMEA output into ROS messages so downstream nodes can consume position cleanly.

Tip for new members: If `/gps_fix_status` will not leave `NONE`, you are likely indoors, the antenna view is obstructed, or the rover is not actually receiving RTCM corrections from the base.

### Basestation GPS driver

#### What is the basestation doing?
We use the u‑blox F9P‑class RTK receiver, namely, [ArduSimple SimpleRTK2B Budget](https://www.ardusimple.com/product/simplertk2b/), that sits at a fixed point and runs a survey‑in to establish a stable reference. While stationary, it produces RTCM3 corrections that let the rover resolve integer ambiguities and achieve centimeter‑level relative accuracy. Why survey‑in matters: it statistically averages the base’s position until it stabilizes, so all subsequent RTCM corrections are referenced to a consistent, low‑noise coordinate; in this codebase, `localization/basestation_gps_driver.py` monitors UBX `NAV‑SVIN` (logs “survey‑in started/complete”) and decodes RTCM 1005 to publish `basestation/position`. If you start using corrections before `NAV‑SVIN.valid` is achieved (or while the reference is drifting), the rover is more likely to remain in `FLOAT` or oscillate; once survey‑in completes, steadier corrections make achieving and holding `FIXED` far more likely.

#### Exactly what our basestation driver does
- Serial input: opens the receiver’s USB CDC‑ACM interface.
- Reads: UBX status plus RTCM3 correction messages produced by the receiver.
- Publishes:
  - `rtcm` (RTCM3 bytes): consumed by the rover process and forwarded into the rover receiver.
  - `basestation/position` (`sensor_msgs/NavSatFix`): the base reference position, stable after survey-in completes.
  - `basestation/satellite_signal` (`mrover/SatelliteSignal`): SNR and constellation view for placement checks.

#### Field indicators of an working basestation

- RTCM corrections present and steady
  - Check `ros2 topic hz /rtcm`: expect a stable non-zero rate (~10 Hz). Bursty, zero, or rapidly fluctuating rates indicate a wiring/config issue.
  - Spot-check with `ros2 topic echo /rtcm`: you should see a non-empty byte payload publishing consistently.
- Survey-in completes and stabilizes reference
  - The basestation logs UBX `NAV-SVIN` with `.active` (in progress) and `.valid` (complete). Once complete, mean accuracy (`meanAcc`) should be small and stable (typical F9P defaults target ≤2 m before declaring valid; tighter is better).
  - After completion, the published `basestation/position` should no longer drift beyond small noise.
- Healthy satellite environment
  - GNSS satellites in use should be comfortably >8 and ideally across multiple constellations (GPS, Galileo, BeiDou, GLONASS, QZSS).
  - Signal levels (C/N0, published via `basestation/satellite_signal`) should be mostly in the ~35–50 dB‑Hz range in open sky, with consistent counts over time. Persistently low SNR (<25 dB‑Hz), frequent dropouts, or large swings suggest poor placement, cabling, or obstructions.
- If unhealthy
  - No `/rtcm` traffic, survey-in never reaches `.valid`, very low satellite count/SNR, or drifting `basestation/position` → re-check antenna sky view, cabling/connectors, ground plane, and basestation configuration.

### Rover GPS driver

#### Our Rover GPS Receiver
We use the [ArduSimple simpleRTK3B Compass](https://www.ardusimple.com/product/simplertk3b-compass/) with the Unicorecomm UM982 dual‑antenna, multi‑band receiver.

Why dual‑antenna: it computes true heading from the fixed baseline vector between two antennas, so orientation remains accurate at low speed and standstill. This improves localization observability.

#### Exactly what our rover driver does
- Subscribes to `rtcm` and writes those bytes to the UM982 over serial.
- Parses NMEA and available status from the rover receiver.
- Publishes:
  - `/gps/fix` (`sensor_msgs/NavSatFix`) — geodetic position for the rest of localization.
  - `/gps_fix_status` (`mrover/FixStatus`) — `NONE | FLOAT | FIXED | NO_SOL`.
  - `/gps/satellite_signal` (`mrover/SatelliteSignal`) — SNR and constellation counts.

#### Physical hookup tips for UM982 dual‑antenna

- Identify and wire the correct ports
  - Use the master (GPS1 / Port A) and slave (GPS2 / Port B) connectors as documented. The heading sign is defined from master → slave. If your reported heading is 180° off, the connectors are swapped or the baseline sign does not match the rover’s forward axis.
- Baseline length and placement
  - Keep antennas level, at the same height, and rigidly mounted. A 1.0 m baseline typically yields ~0.14° heading accuracy; longer baselines improve precision with diminishing returns on a small rover like ours.
  - Maintain at least ~0.6–1.0 m separation when space is limited. Ensure the baseline aligns with your desired heading reference (e.g., rover +X).
- Antennas, cables, and RF hygiene
  - Use matching multiband GNSS antennas (L1/L2/L5 as required by the receiver) with proper ground planes (our current antennas should be mounted on baseplates). Keep coax runs short, low‑loss, and similar in length/quality for both antennas to minimize phase bias.
  - Fully seat and torque SMA connectors; avoid wobbly adapters. Route RF cables away from high‑current wiring and switching supplies to reduce EMI.
- Sky view and multipath
  - Give both antennas an equally unobstructed, symmetric sky view. Keep clear of large metal panels, masts, or reflective surfaces that can cause multipath interference. Aim for consistent C/N0 and satellite counts on both antennas.

#### Further Guidance and Specs
- Product page (UM982): [ArduSimple simpleRTK3B Compass](https://www.ardusimple.com/product/simplertk3b-compass/)
- UM98x configuration guide (UPrecise, commands, base/rover modes): [How to configure Unicore UM980/UM981/UM982](https://www.ardusimple.com/how-to-configure-unicore-um980-um981-um982/)

### RTK

Real-Time Kinematics (RTK) is a technique used to improve the accuracy of satellite-based positioning (like GPS). It uses measurements of the phase of a signal's carrier wave in addition to the content of the signal (GPS data) in order make a positioning observation. In addition, the system relies on real-time corrections coming from a base station or an aggregate of base stations who also measure the phase of carrier wave signals. This allows the units to calculate their relative position to within millimeters, although their absolute position is accurate only to the same accuracy as the computed position of the base station. The typical nominal accuracy for these systems is 1 centimeter ± 2 parts-per-million (ppm) horizontally and 2 centimeters ± 2 ppm vertically.

The range to a satellite is essentially calculated by multiplying the carrier wavelength times the number of whole cycles between the satellite and the rover and adding the phase difference. Determining the number of cycles is non-trivial, since signals may be shifted in phase by one or more cycles. This results in an error equal to the error in the estimated number of cycles times the wavelength, which is 19 cm for the L1 signal. Solving this so-called integer ambiguity search problem results in centimeter precision. The error can be reduced with sophisticated statistical methods that compare the measurements from the coarse-acquisition (C/A) signals and by comparing the resulting ranges between multiple satellites.

<p align="center">
   <img src="https://github.com/user-attachments/assets/037d431c-70c4-4af1-8ce2-39d379c60d37" height="400">
</p>

#### How MRover Uses RTK

Our RTK configuration currently consists of one base station module and one rover module. The easiest way to remember our configuration is as follows, the base station rtk module will be located in the MRover base station and the rover rtk module will be located on the MRover rover.

 - Networking: the base station’s RTCM corrections are sent over ROS 2 on the `rtcm` topic (`rtcm_msgs/Message`) by the `basestation_gps_driver.py` node, and the rover driver subscribes to `rtcm` and writes the bytes over serial directly to the rover’s GPS receiver. The result being that if the base station is working correctly and you can communicate with the rover then the connection between the base station and rover gps modules has already been established. To verify, run on the rover: `ros2 topic echo /rtcm` (or `ros2 topic hz /rtcm`). This will confirm that RTCM messages are being received and that the publish rate is as expected.

 - On the base station, launch `ros2 launch mrover localization_basestation.launch.py` to interface with the base station gps module, the device file for the base station gps module may vary based on what devices are currently plugged in but it is expected to be a `ttyACM` device where a number starting from zero is appended to the device name to specify which device is which. Expect the base station gps filepath to be `/dev/ttyACM0` and you should probably be fine. If you need to change the file path to the mounted device file you can do so by changing the parameters located in `mrover/config/localization.yaml` under `basestation_gps_driver` (keys: `port`, `baud`).

 - On the rover, launch `ros2 launch mrover localization.launch.py` to interface with the rover gps module, the device file for the rover gps module may vary based on the devices that are currently plugged in but will most likely be some variant of `/dev/ttyACM`. Expect the rover gps filepath to be `/dev/ttyACM0` when running on the physical rover; when testing both modules on a single computer, first plug in the basestation and then the rover so the rover gps module is mounted to `/dev/ttyACM1`. If you need to change the expected filepath you can do so in `mrover/config/localization.yaml` under `rover_gps_driver` (use `port_unicore` for Unicore or `port_ublox` for u‑blox).

- Status expectations
  - No corrections available/used:
    - `/gps_fix_status.fix_type` = `NONE` (valid standalone GNSS, no RTK applied) or `NO_SOL` (no valid GNSS position; e.g., indoors or no satellites).
    - Verify: `/rtcm` has no traffic, or rover not subscribed/forwarding; check antenna/sky view.
  - Corrections available but integer ambiguity not resolved:
    - `/gps_fix_status.fix_type` = `FLOAT` (RTK float). Centimeter-to-decimeter level; can fluctuate with signal quality.
    - Verify: `/rtcm` shows traffic; rover sees satellites but baseline/geometry or SNR not sufficient for `FIXED`.
  - Corrections available and integer ambiguity resolved:
    - `/gps_fix_status.fix_type` = `FIXED` (RTK fixed). Centimeter-level relative accuracy.
    - Expect: stable satellite visibility, dual-frequency signals, consistent corrections.
  - Basestation survey-in:
    - Let the basestation complete UBX `NAV-SVIN` (survey-in) for a stable, repeatable reference before expecting rover `FLOAT`/`FIXED`.

#### The Config 

- Location: `mrover/config/localization.yaml`
- Global
  - `world_frame`, `rover_frame`
  - `ref_lat`, `ref_lon`, `ref_alt` (set to the site start to get meaningful ENU meters)
- Basestation (`basestation_gps_driver.ros__parameters`)
  - `port`: e.g., `/dev/ttyACM0`
  - `baud`: e.g., `38400`
- Rover — Unicore default (`rover_gps_driver.ros__parameters`)
  - `port_unicore`: e.g., `/dev/unicore_gps`
  - `baud_unicore`: e.g., `115200`
  - `frame_id`: e.g., `"right_gps_frame"`
- Rover — u‑blox alternative (`rover_gps_driver.ros__parameters`)
  - `port_ublox`: e.g., `/dev/ublox_gps`
  - `baud_ublox`: e.g., `38400`


## IMU
An IMU (Inertial Measurement Unit) is a device that contains several sensors that are used in combination to measure orientation and movement. A 9DoF (Degree of Freedom) IMU like the one we use consists of:
- A 3 axis gyroscope, which can measure angular velocity in each axis
- A 3 axis accelerometer, which can measure linear acceleration in each axis
- A 3 axis magnetometer, which essentially acts as a compass and can measure the direction of magnetic north in each axis

All of this data is then combined using sensor fusion algorithms (more on this later) in order to produce a 3D orientation estimate.

### IMU Driver

We use an [Adafruit BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor) IMU. It is centered around the [Bosch BNO055 chip](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf), which is what contains all of the sensors. This IMU has onboard sensor fusion capabilities, as it can (allegedly) produce a globally accurate 3D orientation measurement using its own onboard black box sensor fusion algorithms. It can provide a wide range of data over I2C and UART. 

<img align="right" src="https://user-images.githubusercontent.com/32557768/189784067-5d05a869-f428-4b57-983a-918a250cebc9.png" width="300" />
<img align="right" src="https://user-images.githubusercontent.com/32557768/192074465-e12b68c7-fe06-442f-b44b-e97e21c5b5af.png" width="300" />

In order to access the data it measures, both the raw sensor readings and the filtered orientation estimate, we obviously need to read them from the sensor into our computer. Since the only good library for this sensor only supports I2C, and ordinary computers can't read I2C, we need an intermediary processor to read it for us and convert the data to serial data that the computer can actually read. For this we are using an [Arduino Nano Every](https://docs.arduino.cc/hardware/nano-every) microcontroller. We have an [Arduino sketch](https://github.com/umrover/embedded-testbench/blob/2023/imu/imu/bno055_serial/bno055_serial.ino) kept in the [embedded-testbench repository](https://github.com/umrover/embedded-testbench) which uses the [Adafruit BNO055 Unified Sensor Library](https://github.com/adafruit/Adafruit_BNO055) to read the IMU data over I2C and then publish it over a serial connection. The arduino is then plugged into our main Jetson computer over USB so the data is accessible on a serial port.

To get this data from serial to the ROS network, we have an IMU driver node. this node reads the IMU data over serial and then publishes it to several standard ROS messages, which other nodes can then subscribe to. The IMU driver node is configured in `config/esw.yaml`. The specific data published includes:
- IMU: [`Imu`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) messages on the `/imu/data` topic
- Magnetometer: [`MagneticField`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) messages on the `/imu/magnetometer` topic
- IMU Temperature: [`Temperature`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html) messages on the `/imu/temp` topic
- IMU Calibration: custom `CalibrationStatus` messages on the `/imu/calibration` topic

For information on the design process and decisions behind the IMU driver, read [this discussion](https://github.com/umrover/mrover-ros/discussions/167).

## Cameras

We currently use the [ZED 2i](https://www.stereolabs.com/products/zed-2). Existing algorithms have the capability of detecting visual features seen by the camera and matching them across frames, allowing us to determine how they have shifted over time. We can then use this information to derive the camera's movement and estimate pose in a technique known as visual odometry.

![feature_matching](https://github.com/user-attachments/assets/a32186d4-9885-44ec-9c2f-0f68e340db31)

Functions such as the [8-Point Algorithm](https://docs.opencv.org/4.x/da/de9/tutorial_py_epipolar_geometry.html) can identify the rotation and translation matrices $R$ and $T$ from 8 (as its name suggests) matched features. Put together, these produce a `odom->base_link` transform that describes pose.


# GPS Linearization
The geodetic coordinates (latitude, longitude, altitude) that the GPS gives us are very hard to work with, since they can be very inconvenient and unintuitive over small distances. To solve this problem, we can linearize them into cartesian coordinates (x, y, z) using an ENU local tangent plane approximation.

<p align="center">
<img src="https://user-images.githubusercontent.com/32557768/217405374-16591569-3816-49f1-be41-2e0c1d056bdb.png" width="450" />
</p>


This approximation essentially assumes the earth is flat in the area where the rover is driving. In our case this is actually a pretty good approximation, since the rover only ever drives a few kilometers during the auton mission, and this distance is very small relative to the size of the earth.

The linearization is computed by first converting the geodetic coordinates from the GPS to ECEF (earth centered, earth fixed) coordinates using the [WGS 84](https://en.wikipedia.org/wiki/World_Geodetic_System) ellipsoid model of the earth. These ECEF coordinates are cartesian coordinates, but their origin (0,0,0) is at the center of the earth. These coordinates are then transformed to be centered at the tangent plane around the rover. The equations for this transform can be found [here](https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU). 

The local tangent plane is centered at a reference geodetic coordinate, which is a (latitude, longitude, altitude) coordinate that we choose which is close enough to the area of operation that the linearization approximation is accurate. This means __the reference coordinate needs to be changed__ when the rover is being operated in very different geographical location, such as when we move from Michigan to Utah for our competition.

## Accuracy
It's difficult to measure the accuracy of the tangent plane linearization, since we don't have any pairs of ground truth geodetic coordinates that are a known distance apart (maybe you could find some of these online). What we've done to measure accuracy is use [this calculator](https://www.gpsvisualizer.com/calculators) to calculate distance between geodetic coordinates (that were around 30 km apart) and compare that calculated distance to the distance calculated through linearization. The calculator uses [Vincenty's Formulae](https://en.wikipedia.org/wiki/Vincenty%27s_formulae) to calculate distance, which is accurate within a millimeter. Unfortunately the calculator is only precise to 1 meter, so the only hard accuracy bound we can give is __within 1 meter across 30km__. If we want a tighter bound, we could find a higher precision calculator that uses the same formula and compare to that.

Having a tight accuracy bound probably isn't all that important, because small inaccuracies probably won't be reflected in our system's actual performance. This is because our target geodetic waypoints are linearized using the same algorithm, so effectively both our measured position and our target position are being mapped through the same function. The properties of this linearization function mean that even if it maps our current GPS geodetic coordinate to an incorrect cartesian coordinate, it will map our target coordinate to an incorrect cartesian coordinate in the same way. This means our distance to the target will still approach zero as we get closer to the target, and the distance between position and target will have all of the properties required for our navigation to work, even if the actual distances are inaccurate.

# Guide to Localization Frames
When determining and defining where the robot is, we have to deal with an inherent tradeoff in data quality. Usually data sources can be either locally accurate or globally accurate, but it is much more difficult to get a single source of data that is both globally and locally accurate.

This problem is addressed by the standards introduced in [REP 105](https://www.ros.org/reps/rep-0105.html) (which you should read to get a general overview). Unfortunately, the REP uses some misleading terms and gives a somewhat confusing explanation, so here is a hopefully more clear way of explaining it.

First, a few important notes and definitions:

## local vs global accuracy
In the context of localization, we often talk about local and global accuracy, so we need to be very clear about what these things mean. Locally accurate data must change in a smooth way, i.e. no discrete/discontinuous jumps. However, it may drift over time without bounds, meaning it can gradually accumulate large errors. On the other hand, globally accurate data must not drift significantly over time, but may have discrete/discontinuous jumps at any time. These definitions are of course a little bit vague, since they somewhat depend on the idea of a short vs long period of time, but this isn't usually a problem since we don't deal with a lot of significant edge cases.

## abstraction of sensor data sources into transforms
For the purposes of this explanation, we will abstract away any sensor processing/fusion algorithms and just imagine all localization data sources as providing a transform telling us where the robot is located. A global sensor transform will be relative to a fixed frame, in this case the `map` frame. A local sensor transform on the other hand may be relative to any arbitrary starting point, so long as the transform obeys the rules of local accuracy.

<p align="center">
<img src="https://user-images.githubusercontent.com/32557768/176823827-a4c61cc4-6ba2-439a-bf2e-8cc24bcfda09.png" width=1000>
</p>

### `map`
Since pose is relative (see SE3 wiki entry), we need to define where the robot is relative to some "fixed" frame. Fixed in this case means it does not move relative to the thing/place we are navigating in, which we will call the world. We will use the "map" frame as our fixed frame, and the pose of the robot will be defined relative to that frame, i.e. it will be defined as a transform from map to the robot.

### `base_link`
The base_link frame is simply the frame of the robot. It is rigidly attached to the robot, and in our case is located at the center of the chassis.

### `odom`
The odom frame is an intermediate frame in between map and base_link. It doesn't really have a physical representation, but it gives us a good way to separate local and global data sources. Odom essentially acts as a local reference frame for the robot. This means that the pose of the robot relative to the odom frame should always be locally accurate, but doesn't need to be globally accurate.

There are two transforms we need to define in order to connect these three frames, which thereby defines the pose of the robot:

### `odom_to_base_link`
The transform from the odom frame to the base_link frame, which we will call odom_to_base_link, will be defined using locally accurate sensor data. This means the pose of the robot in the odom frame (which is the exact same thing as this transform) will be locally accurate, i.e it can drift over time, but must always change smoothly and continuously without discrete jumps.

### `map_to_odom`
The transform from the map frame to the odom frame, which we will call map_to_odom in this case, will be defined using globally accurate sensor data. Our goal here is to use this data to make the transform from map to base_link (which is equal to map_to_odom * odom_to_base_link) globally accurate, i.e it will not drift over time, but it may change non-continuously with discrete jumps. We want to do this by only changing the map_to_odom frame, which means we have to first figure out what the odom_to_base_link transform is (by asking the TF tree) and then "subtract" that from our global sensor transform in order to determine the correct map_to_odom transform.

<p align="center">
<img src="https://user-images.githubusercontent.com/32557768/213590093-a23a69be-c96f-4663-a0fc-272a5dcbd48f.png" width="600">
</p>



## Using these frames in practice
The main benefit of this system is having an organized way of separately accessing local and global localization data. For applications where you need a locally accurate localization source, you can get the `odom_to_base_link` transform from the TF tree and use that as your robot pose. Similarly, for applications where you need globally accurate localization, you can get the `map_to_odom` transform from the TF tree and use it as your robot pose (for exact syntax, see the wiki section about the TF tree or just google it).

Here's a simple example of using these localization frames: suppose we have a simple 4 wheeled robot equipped with a GPS unit and wheel encoders. The GPS provides a globally accurate source of localization data, while the wheel encoder data can be fed into a sensor processing algorithm to produce a locally accurate source of localization data. 

In order to use these two data sources, our first step is to publish the wheel encoder localization data as the `odom_to_base_link` transform to the TF tree. Similarly, we will also use the GPS data to publish the `map_to_odom` transform to the TF tree, as described in the `map_to_odom` section. Once this is done, we can actually use this data for some autonomous routines.

Suppose we want to write a function that rotates the robot 90 degrees in place. Since this is a routine that won't take very much time, is only based on relative measurements, and needs to be quite accurate on a local scale, we want to use a locally accurate data source. To do this, we will get the `odom_to_base_link` transform from the TF tree, look it's rotation, and then send the robot drive commands until the rotation we read from `odom_to_baselink` has increased by 90 degrees.

Now suppose we want to write a function to drive to a specific waypoint on our map, a mile away. Since this routine will take a while and is based on absolute measurements (relative to the map frame), we need to use a globally accurate data source to avoid drift and so we can measure relative to the map frame. To do this, we will get the `map_to_odom` transform from the TF tree, look at its position, and then use a drive controller to drive the robot in the direction of the waypoint until the position we read from `map_to_odom` is close enough to our target waypoint.

# Kalman Filters and Sensor Fusion

A Kalman filter is a dynamic low-pass filter that reduces noise in sensor measurements. It is a 2-phase recursive algorithm consisting of a prediction step and estimation step, allowing it to accept many inputs at various points for the best possible estimation of the system's state. This kind of sensor fusion is integral to the localization subsystem to provide accurate pose information. 

A general algorithm flowchart is shown below:

<img width="660" alt="Screenshot 2024-08-24 at 3 29 28 PM" src="https://github.com/user-attachments/assets/a198556f-1297-4e62-a0d6-0263d732404c">

 - Inputs: sensor measurement(s) ($z$, n x 1 column matrix). Can be obtained from a variety of sources. For example, magnetometer and accelerometer data are often fused to produce a 3D orientation measurement.

 - Outputs: state variable ($x_k$, m x 1 column matrix). What you want to estimate - this does not have to be the same type as the input. For example, a very common use of the Kalman filter takes in position (measurement) to estimate velocity (state).

 - System model: Parameters for the Kalman filter (matrices $A$, $H$, $Q$, $R$). Dependent on the process being estimated and amount of noise in sensors. More on this below.

 - Additional Notes: hat symbol ($\hat{x_k}$) denotes estimation, minus symbol (${\hat{x_k}}^{-}$) denotes prediction.

## Determining the System Model

At its core, the Kalman filter is a recursive function. We use this fact to determine the system model matrices $A$, $H$, $Q$, and $R.$

Linear state estimation is defined recursively by:

## $x_k = Ax_{k-1} + w_k$

Where $w_k$ is the state error and $A$ is the n x n state transition matrix. Because the state estimation is linear, it follows that no elements within $A$ may depend on the state variable $x_k$. An extended Kalman filter (EKF) is a generalized version of the Kalman filter that allows for nonlinear state estimation.

Sensor measurement is defined by:

## $z = Hx_k + v_k$

Where $v_k$ is the measurement error, and $H$ is the m x n matrix that maps state to measurement.

$Q$ and $R$ are the process covariance and sensor covariance matrices respectively. These are sometimes given by the manufacturer, otherwise they are tuned from the information given on the sensor's noise level and educated guesses on how it affects the state variable.

## Prediction and Estimation

After providing the filter with an initial state and error covariance, the prediction and estimation steps loop to continuously receive new data and update the state estimate. 

### Prediction

<img height="100" alt="Screenshot 2024-08-26 at 10 58 54 AM" src="https://github.com/user-attachments/assets/242c0bee-b64f-4d93-8e5d-e0939eff95b6"><br>

The algorithm predicts the state estimate and the error covariance using the above equations. Notice that the equation used to predict the state estimate is identical to the linear state estimation function, only without the error term.

The error covariance is instead computed separately and makes use of the error matrix $Q$. 

### Estimation

<img height="50" alt="Screenshot 2024-08-26 at 11 01 36 AM" src="https://github.com/user-attachments/assets/d0be7378-720a-436c-a100-1916497fad1e"><br>
<img height="50" alt="Screenshot 2024-08-26 at 11 01 42 AM" src="https://github.com/user-attachments/assets/d8d40f04-ef6b-4fa0-8bda-cdd9dc50495b"><br>
<img height="50" alt="Screenshot 2024-08-26 at 11 01 47 AM" src="https://github.com/user-attachments/assets/c173d453-5d80-4201-9adf-7de8efa69929"><br>


First, the Kalman Gain $K_k$ is computed. This determines how much “weight” the algorithm gives incoming measurements. The more weight, the more the incoming measurement influences the state estimate. 

The estimate is computed using the state prediction, Kalman gain, and the difference between the measurement and predicted measurement $H{\hat{x_k}}^{-}$. The error covariance is also calculated, and will be used to predict the ${k + 1}^{th}$ error covariance. 

## Global EKF

To get a globally accurate estimate of the rover's pose, we use an EKF that fuses GPS and IMU data. The EKF outputs a more accurate, less noisy pose estimate which is then published to the TF tree effectively as the `map->base_link` transform.

## Additional Resources

Kalman Filters and Sensor Fusion:

https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
https://www.youtube.com/watch?v=HCd-leV8OkU <br>
https://www.youtube.com/watch?v=qCZ2UTgLM_g <br>
https://www.youtube.com/watch?v=DbE4PMgqp3s <br>
https://www.mathworks.com/videos/sensor-fusion-part-2-fusing-a-mag-accel-and-gyro-to-estimate-orientation-1569411056638.html <br>
Helpful group project from ROB 530: https://www.youtube.com/watch?v=WHBgSqRPpu4
https://arxiv.org/pdf/2311.04320

## Launch Descriptions

- Basestation bring‑up
  - Launch: `ros2 launch mrover localization_basestation.launch.py`
  - Hardware: [ArduSimple SimpleRTK2B Budget](https://www.ardusimple.com/product/simplertk2b/) (u‑blox F9P) RTK GNSS receiver, connected via USB serial (typically `/dev/ttyACM0`).
  - Node: `basestation_gps_driver.py`
    - Reads UBX+RTCM3; logs Survey‑In (`NAV‑SVIN`), GNSS status (`NAV‑PVT`), satellites (`NAV‑SAT`).
    - Publishes:
      - `rtcm` (`rtcm_msgs/Message`) — corrections to rover
      - `basestation/position` (`sensor_msgs/NavSatFix`) — reference position (decoded from RTCM 1005)
      - `basestation/satellite_signal` (`mrover/SatelliteSignal`) — per‑constellation SNRs
  - Configure in `mrover/config/localization.yaml` under `basestation_gps_driver` (`port`, `baud`).

- Rover bring‑up
  - Launch: `ros2 launch mrover localization.launch.py`
  - Hardware: [ArduSimple simpleRTK3B Compass](https://www.ardusimple.com/product/simplertk3b-compass/) (Unicorecomm UM982, dual‑antenna, triple‑band) RTK GNSS receiver, connected via USB serial on the Jetson (typically `/dev/ttyACM*` when you SSH into the rover).
  - Node: `rover_gps_driver` (C++, Unicore path)
    - Subscribes: `/rtcm` (forwards corrections to the receiver over serial)
    - Publishes: `/gps/fix` (`sensor_msgs/NavSatFix`), `/gps_fix_status` (`mrover/FixStatus`), `/gps/satellite_signal` (`mrover/SatelliteSignal`)
    - Configure in `mrover/config/localization.yaml` under `rover_gps_driver` (`port_unicore`, `baud_unicore`, `frame_id`)
  - Unicore UM98x configuration guide (advanced): [How to configure Unicore UM980/UM981/UM982](https://www.ardusimple.com/how-to-configure-unicore-um980-um981-um982/)

- Device rules (stable serial names)
  - Udev rules create:
    - `/dev/unicore_gps` for Unicore N4
    - `/dev/ublox_gps` for u‑blox F9P
  - See `ansible/roles/esw/files/rules/99-gps.rules`.

