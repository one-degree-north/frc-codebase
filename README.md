# Team 4817 Codebase

This is a repo containing the current, most up-to-date codebase for FRC Team 4817.
This codebase is used as a foundation when programming competition robots

The only files you will ever need to edit when using the codebase is `RobotContainer.java` and `Constants.java`

# Dependencies
The following libraries need to be downloaded, extracted, and installed
- [WPILib 2022](https://github.com/wpilibsuite/allwpilib/releases/tag/v2022.1.1)
- [CTRE-Phoenix](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v5.20.2.2)

The following libraries are the JSON files for online installation
- [REVRobotics](https://software-metadata.revrobotics.com/REVLib.json)
- [navX](https://www.kauailabs.com/dist/frc/2022/navx_frc.json)
- [SwerveDriveSpecialties](https://raw.githubusercontent.com/SwerveDriveSpecialties/swerve-lib/master/SdsSwerveLib.json)

# Included Files (`frc.lib`)
## Gyros
`ODN_Gyro` is the basic gyro interface which all gyros implement to allow easy switiching between different gyros in code
- `ODN_AHRS` implements `ODN_Gyro` for the [navX MXP sensor](https://pdocs.kauailabs.com/navx-mxp/)
- `ODN_Pigeon` implements `ODN_Gyro` for the [PigeonIMU sensor](http://www.ctr-electronics.com/gadgeteer-imu-module-pigeon.html)
  - Constructor takes the CAN id of the PigeonIMU
- If you do not have a gyro, but a subsystem requires one in the constructor, use `ODN_NullGyro`
  - Be careful that you don't do anything that would require an actual gyro in this situation

Only functions you will ever need to use:
- `Rotation2d getYaw()` gets the rotation of the robot, positive being clockwise.

## Motor Controllers
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

## Encoders
`ODN_Encoder` is the basic gyro interface which all encoders implement to allow easy switiching between different encoder in code
- `ODN_CANCoder` implements `ODN_Encoder` for the [CANCoder magnetic encoder](http://www.ctr-electronics.com/cancoder.html)
- `ODN_CANEncoder` implements `ODN_Encoder` for the SparkMax integrated encoder
- `ODN_TalonEncoder` implements `ODN_Encoder` for a Talon (TalonSRX or TalonFX) integrated encoder
- `ODN_VictorEncoder` implements `ODN_Encoder` for a Victor integrated encoder
- If you do not have an encoder, but a subsystem requires one in the constructor, use `ODN_NullEncoder`
  - Be careful that you don't do anything that would require an actual encoder in this situation

Only functions you should ever need to use:
- `void setPositionConversionFactor(double factor)` sets the factor for converting from raw encoder output to a meaurement in degrees
-	`void setVelocityConversionFactor(double factor)` sets the factor for converting from raw encoder velocity to a meaurement in degrees (these numbers should probably both be the same, but you do have to set both)

## Sensors
`ODN_UltrasonicSensor` is the basic interface for ultrasonic sensors. There are two types of ultrasonic sensors in FRC, ping-response ultrasonic sensors, which use two seperate DIO pins for sending the ultrasonic signals and measuring the response. This is used in the codebase with the `ODN_PingUltrasonicSensor` class. There are also analog ultrasonic sensors which return an analog voltage corresponding to the measured distance. This is used in the codebase with the `ODN_AnalogUltrasonicSensor` class.

The only function of this class is `double getDistanceInches()` which returns the distance in inches.

`ODN_ColorSensor` is a class for the REV Robotics Color Sensor V3. There are two features with this sensor. Getting the sensed color with `Color getColor()`, and matching a color to a set of possible colors with `Color match(ColorMatch matcher)`. The color matcher object can be created with the `ColorMatch ODN_ColorSensor::createMatcher(double confidence, Color... colors)`. `colors` is a list of colors that you want to compare against. `confidence` is a number between 0-1 which you will need to tune in order to detect when none of the given colors are present.

## Drivebases
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

## ODN_State
The `ODN_State` interface is used for situations in which a motor needs to move itself to certain positions.

Functions you will need to use:
- `void setGoalLocation(double pos)` sets the goal location for the state subsystem. Units for this parameter are not specified, and will depend on a `factor` constant passed into one of `ODN_State`'s subclasses. 
- `boolean atGoalLocation()` returns true if the subsystem has reached the set goal location.
- `void resetPosition()` used for calibration, it sets the current position to zero.

# Included Files (`frc.robot.subsystems`)
## Arm Subsystem
This subsystem is for controlling the position of an arm. Look at the information for `ODN_State` on how to interact with this subsystem. There are no other functions specific to this subsystem.
## Elevator Subsystem
This subsystem is for controlling the position of an elevator. Look at the information for `ODN_State` on how to interact with this subsystem. There are no other functions specific to this subsystem.
## FalconMusicSubsystem
This subsystem is for playing music on the robot with Falcon motors. Remember that when falcons are playing music, they cannot be used for motion.

Functions you may need:
- `void loadMusic(String filename)` loads music from a file from the given filename
- `void play()` plays music. Music needs to be loaded before it can be played
- `void pause()` pauses the music
- `void stop()` stops the music
## LimelightSubsystem
This is a wrapper subsystem for the limelight, simply for easy access to certain values.

Here are the functions you may need to use in this subsystem:
- `foundTarget()` returns true if the limelight has located a target, false otherwise
- `getOffsetHorizontal()` returns the horizontal position of the located target in degrees (or 0 if no target is located)
- `getOffsetVertical()` returns the vertical position of the located target in degrees (or 0 if no target is located)
- `getArea()` returns the fraction (between 0 to 1) of the camera view that is covered by the located target (or 0 if no target is located)
- `getSkew()` returns the angle in degrees that the located target is rotated around the axis of viewing (or 0 if no target is located)
- `setLED(boolean on)` turn the limelight lights on or off
- `setPipeline(int pipeNum)` choose which detection pipeline to use with the limelight
## MecanumDriveSubsystem
A subsystem to control a mecanum drivebase. There is no function specific to this subsystem that you would need to use. See `ODN_HolonomicDrivebase` and `ODN_Drivebase` for functions this subsystem has.
## MotorControllerSubsystem
Literally just a subsystem with a motor controller whose speed you can set
## ParallelStateSubsystem
Used when two `ODN_State` subsystems need to move in unison. This should either be arms or elevators. Don't move an arm or an elevator in unison. The functions for this subsystem are the same as that of `ODN_State`
## PneumaticSubsystem
Subsystem to control a group of pneumatic pistons together. Only works with `DoubleSolenoid`.

Functions you may need:
- `void toggle()` switches the direction of the pistons (forward->backwards or backwards->forwards)
## SwerveDriveSubsystem
A subsystem to control a swerve drivebase. There is no function specific to this subsystem that you would need to use. See `ODN_HolonomicDrivebase` and `ODN_Drivebase` for functions this subsystem has.
## TankDriveSubsystem
A subsystem to control a mecanum drivebase.

Functions you may need:
- `void tankDrive(double left, double right)` drives using the left and right speeds

# Included Files (`frc.robot.commands`)

You may need to add some of your own commands into this folder. Below is an explanation of the commands that are currently a part of the codebase.

## ArmCommand
This command uses an arm placed on an vertical elevator, moving the end of the arm horizontally at constant speed
## LimelightArcCommand
This command locks the robot into only moving in an arc around a piece of reflective tape at a constant distance. This only works with `ODN_HolonomicDrivebase`, so no tank drive with this command.
## TrajectoryCommand
This command makes the robot follow a trajectory, either given with position data or as a PathPlanner path file.
