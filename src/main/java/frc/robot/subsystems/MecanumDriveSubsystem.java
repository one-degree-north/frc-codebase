// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ODN_HolonomicDrivebase;
import frc.lib.encoder.Encoder;
import frc.lib.motorcontroller.MotorControllerGroup;
import frc.robot.Constants.AutoConstants;

public class MecanumDriveSubsystem extends SubsystemBase implements ODN_HolonomicDrivebase {
  public static class Constants {
    // MotorControllers
    public MotorControllerGroup frontLeft;
    public MotorControllerGroup rearLeft;
    public MotorControllerGroup frontRight;
    public MotorControllerGroup rearRight;

    // Encoders
    public Encoder frontLeftEncoder;
    public Encoder rearLeftEncoder;
    public Encoder frontRightEncoder;
    public Encoder rearRightEncoder;

    // CAN ID for gyro
    public int id_gyro;

    /** Converts output from encoder to meters travelled on the left side */
    public double frontLeftEncoderFactor;
    public double rearLeftEncoderFactor;
    /** Converts output from encoder to meters travelled on the right side */
    public double frontRightEncoderFactor;
    public double rearRightEncoderFactor;

    // These three are factors for the feedforward calculations
    public double ksVolts;
    public double kvVoltSecondsPerMeter;
    public double kaVoltSecondsSquaredPerMeter;

    public MecanumDriveKinematics kDriveKinematics;

    /**
     * PID proportional factor for Trajectories. Should be set to one since velocity
     * is in meters per second
     */
    public double kPDriveVel = 1;
  }

  private Encoder frontLeftEncoder;
  private Encoder rearLeftEncoder;
  private Encoder frontRightEncoder;
  private Encoder rearRightEncoder;

  private MotorControllerGroup frontLeft;
  private MotorControllerGroup rearLeft;
  private MotorControllerGroup frontRight;
  private MotorControllerGroup rearRight;

  private MecanumDrive drive;

  public Constants m_constants;

  private PigeonIMU gyro;

  private MecanumDriveOdometry m_odometry;

  /**
   * Constructor for the TankDrive subsystem.
   * 
   * @param constants Object containing all the constants for this drivebase
   */
  public MecanumDriveSubsystem(Constants constants) {
    this.m_constants = constants;
    frontLeft = constants.frontLeft;
    rearLeft = constants.rearLeft;
    frontRight = constants.frontRight;
    rearRight = constants.rearRight;
    drive = new MecanumDrive(frontLeft.getBackend(), rearLeft.getBackend(), frontRight.getBackend(), rearRight.getBackend());
    gyro = new PigeonIMU(constants.id_gyro);
    m_odometry = new MecanumDriveOdometry(constants.kDriveKinematics, Rotation2d.fromDegrees(getYaw()));

    frontLeft = constants.frontLeft;
    rearLeft = constants.rearLeft;
    frontRight = constants.frontRight;
    rearRight = constants.rearRight;

    frontLeftEncoder.setPositionConversionFactor(constants.frontLeftEncoderFactor);
    rearLeftEncoder.setPositionConversionFactor(constants.rearLeftEncoderFactor);
    frontRightEncoder.setPositionConversionFactor(constants.rearRightEncoderFactor);
    rearRightEncoder.setPositionConversionFactor(constants.rearRightEncoderFactor);
    frontLeftEncoder.setVelocityConversionFactor(constants.frontLeftEncoderFactor);
    rearLeftEncoder.setVelocityConversionFactor(constants.rearLeftEncoderFactor);
    frontRightEncoder.setVelocityConversionFactor(constants.frontRightEncoderFactor);
    rearRightEncoder.setVelocityConversionFactor(constants.rearRightEncoderFactor);

  }

  /**
   * Periodic function for TankDrive subsystem. Updates the odometry.
   * 
   * @see DifferentialDriveOdometry#update(Rotation2d, double, double)
   */
  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(),
        frontRightEncoder.getVelocity(), rearLeftEncoder.getVelocity(), rearRightEncoder.getVelocity()));
  }

  /**
   * Gets rotation around z-axis from the PigeonIMU. Positive is clockwise.
   * Measurement is in degrees.
   * 
   * @return rotation around z-axis in degrees
   */
  public double getYaw() {
    double[] arr = new double[3];
    gyro.getYawPitchRoll(arr);
    return arr[0];
  }

  /**
   * Drives the robot using the arcade drive mode
   * 
   * @param forward  The robot's speed along the x axis [-1, 1]. Forward is
   *                 positive.
   * @param rotation The robot's turning rate around the z axis [-1, 1]. Clockwise
   *                 is positive.
   */
  public void cartesianDrive(double ySpeed, double xSpeed, double xRotation) {
    drive.driveCartesian(ySpeed, xSpeed, xRotation);
  }

  /**
   * Drives the robot using the tank drive mode
   * 
   * @param left  The robot's left side speed along the X axis [-1, 1]. Forward is
   *              positive.
   * @param right The robot's right side speed along the X axis [-1, 1]. Forward
   *              is positive.
   */
  public void polarDrive(double magnitude, double angle, double zRotation) {
    drive.drivePolar(magnitude, angle, zRotation);
  }

  /**
   * 
   * @return The current position of the robot
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry
   * 
   * @param pose The new pose according to the new coordinate plane
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
  }

  /**
   * Resets the left and right encoder positions to zero
   */
  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }

  /**
   * 
   * @return the speeds of both sides of the robot
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(), 
    rearLeftEncoder.getVelocity(), 
    frontRightEncoder.getVelocity(),
    rearRightEncoder.getVelocity());
  }

  /**
   * Drives the robot using the tank drive mode with input in volts
   * 
   * @param leftVolts  Voltage to send to left side motors
   * @param rightVolts Voltage to send to right side motors
   */
  public void mecanumDriveVolts(MecanumDriveMotorVoltages volts) {
    frontLeft.setVoltage(volts.frontLeftVoltage);
    rearLeft.setVoltage(volts.rearLeftVoltage);
    frontRight.setVoltage(volts.frontRightVoltage);
    rearRight.setVoltage(volts.rearRightVoltage);
    drive.feed();
  }

  @Override
  public Command generateTrajectoryCommand(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
    var feedforward = new SimpleMotorFeedforward(m_constants.ksVolts, m_constants.kvVoltSecondsPerMeter,
        m_constants.kaVoltSecondsSquaredPerMeter);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to correctly calculate wheel speeds
            .setKinematics(m_constants.kDriveKinematics);

    // The trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Starting pose
        startPose,
        // List of waypoints to pass through
        waypoints,
        // Ending pose
        endPose,
        // Pass config to generate Trajectory properly for this drivebase
        config);

        var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new MecanumControllerCommand(
        // Trajectory to travel
        trajectory,
        // RamseteCommand needs to be able to 
        // access the pose of the robot at any time
        this::getPose,
        // Feedforward to ensure correct voltage is sent to motors
        feedforward,
        // Kinematics to calculate correct wheel speeds
        m_constants.kDriveKinematics,
        // PID Controllers used to compensate for external factors in motor speed
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        AutoConstants.kMaxSpeedMetersPerSecond,
        new PIDController(m_constants.kPDriveVel, 0, 0),
        new PIDController(m_constants.kPDriveVel, 0, 0),
        new PIDController(m_constants.kPDriveVel, 0, 0),
        new PIDController(m_constants.kPDriveVel, 0, 0),
        this::getWheelSpeeds,
        // RamseteCommand passes volts to the callback to drive the wheels
        this::mecanumDriveVolts,
        this
    );
  }
}
