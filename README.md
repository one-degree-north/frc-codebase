# Team 4817 Codebase

This is a repo containing the current, most up-to-date codebase for FRC Team 4817.
This codebase is used as a foundation when programming competition robots

The only files you will ever need to edit when using the codebase is `RobotContainer.java` and `Constants.java`

## Dependencies
- [WPILib 2022](https://github.com/wpilibsuite/allwpilib/releases/tag/v2022.1.1-rc-1)
- [REVRobotics](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#java-api)
- [CTRE-Phoenix](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v5.20.2.2)

## Included Files (`frc.lib`)
### Gyros
`ODN_Gyro` is the basic gyro interface which all gyros implement to allow easy switiching between different gyros in code
- `ODN_AHRS` implements `ODN_Gyro` for the [navX MXP sensor](https://pdocs.kauailabs.com/navx-mxp/)
- `ODN_Pigeon` implements `ODN_Gyro` for the [PigeonIMU sensor](http://www.ctr-electronics.com/gadgeteer-imu-module-pigeon.html)
  - Constructor takes the CAN id of the PigeonIMU
- If you do not have a gyro, but a subsystem requires one in the constructor, use `ODN_NullGyro`
  - Be careful that you don't do anything that would require an actual gyro in this situation

Only functions you will ever need to use:
- `Rotation2d getYaw()` gets the rotation of the robot, positive being clockwise.

### Motor Controllers
`ODN_MotorController` is the basic motor controller interface which all motor controllers implement to allow easy switching between different motor controllers in code
- `ODN_SparkMax` implements `ODN_MotorController` for the [SparkMax](https://www.revrobotics.com/rev-11-2158/) connected using the CAN bus
  - Constructor second parameter is either `ODN_SparkMax.MotorType.brushed` for brushed motor configuration or `ODN_SparkMax.MotorType.brushless` for brushless motor configuration or 
- `ODN_TalonFX` implements `ODN_MotorController` for the [TalonFX (Falcon 500)](http://www.ctr-electronics.com/talon-fx.html)
- `ODN_TalonSRX` implements `ODN_MotorController` for the [TalonSRX](https://www.ctr-electronics.com/talon-srx.html)
- `ODN_VictorSPX` implements `ODN_MotorController` for the [VictorSPX](http://www.ctr-electronics.com/victor-spx.html)

Constructors for all the motor controllers takes the CAN id of the motor controller

Only functions you will ever need to use:
- `void setInverted(boolean inverted)` is used to reverse the direction of the motor controller
- `Encoder getEncoder()` is used to get the integrated encoder from the motor controller

### Encoders
`Encoder` is the basic gyro interface which all encoders implement to allow easy switiching between different encoder in code
- `ODN_CANCoder` implements `Encoder` for the [CANCoder magnetic encoder](http://www.ctr-electronics.com/cancoder.html)
- `ODN_CANEncoder` implements `Encoder` for the SparkMax integrated encoder
- `ODN_TalonEncoder` implements `Encoder` for a Talon (TalonSRX or TalonFX) integrated encoder
- `ODN_VictorEncoder` implements `Encoder` for a Victor integrated encoder
- If you do not have an encoder, but a subsystem requires one in the constructor, use `ODN_NullEncoder`
  - Be careful that you don't do anything that would require an actual encoder in this situation

Only functions you should ever need to use:
- `void setPositionConversionFactor(double factor)` sets the factor for converting from raw encoder output to a meaurement in degrees
-	`void setVelocityConversionFactor(double factor)` sets the factor for converting from raw encoder velocity to a meaurement in degrees (these numbers should probably both be the same, but you do have to set both)

### Drivebases
In `frc.lib` there exists an `ODN_Drivebase` class and an `ODN_HolonomicDrivebase` class. Using `ODN_Drivebase` allows for switching drivebases more easily in the code. However, not all drivebases are made equal. A tank drive cannot move sideways. This is what we have `ODN_HolonomicDrivebase` which generalizes Swerve Drive and Mecanum Drive (but could also include Octocanum drive in the future along with other similarly abled drivebases).


Functions you may need to use from `ODN_Drivebase`:
- `void rotate(double speed)` rotates in place
- `void stop()` stops the robot in place
- `Rotation2d getYaw()` gets the orientation of the robot (don't use this function if you are using `ODN_NullGyro` for this drivebase)
- `void resetYaw()` sets the current heading to 0 degrees
- `void driveForward(double forward, double rotate)` drives the robot forward and rotates (arcade drive)
- `void resetOdometry(Pose2d initialPose)` resets the odometry, defining the current pose to be `initialPose` (odometry only works if you have all the nessecary encoders and gyro on the drivebase, e.g. no `ODN_NullEncoder` or `ODN_NullGyro`)

Functions you may need to use from `ODN_HolonomicDrivebase`:
- `void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate)` drives with velocity given in cartesian coordinates field-relative
  - Cannot be used without a real gyro (not `ODN_NullGyro`)
- `void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate)` drives with velocity given in cartesian coordinates robot-relative
- `void polarDrive(double magnitude, Rotation2d direction, double rotate)` drives with velocity given in polar coordinates
  - Cannot be used without a real gyro (not `ODN_NullGyro`)