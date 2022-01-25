// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.lib.ODN_Drivebase;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.gyro.ODN_Gyro;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.robot.Constants.AutoConstants;

public class TankDriveSubsystem extends ODN_Drivebase {
  public static class Constants {
    // ODN_MotorControllers
    public ODN_MotorControllerGroup left;
    public ODN_MotorControllerGroup right;

    // Encoders
    public ODN_Encoder leftEncoder;
    public ODN_Encoder rightEncoder;

    // gyro
    public ODN_Gyro gyro;

    /** Converts output from encoder to meters travelled on the left side */
    public double leftEncoderFactor;
    /** Converts output from encoder to meters travelled on the right side */
    public double rightEncoderFactor;

    // These three are factors for the feedforward calculations
    public double ksVolts;
    public double kvVoltSecondsPerMeter;
    public double kaVoltSecondsSquaredPerMeter;

    public DifferentialDriveKinematics kDriveKinematics;

    /**
     * PID proportional factor for Trajectories. Should be set to one since velocity
     * is in meters per second
     */
    public double kPDriveVel = 1;
  }

  private ODN_Encoder leftEncoder;
  private ODN_Encoder rightEncoder;

  private ODN_MotorControllerGroup left;
  private ODN_MotorControllerGroup right;

  private DifferentialDrive drive;

  public Constants m_constants;

  private DifferentialDriveOdometry m_odometry;

  /**
   * Constructor for the TankDrive subsystem.
   * 
   * @param constants Object containing all the constants for this drivebase
   */
  public TankDriveSubsystem(Constants constants) {
    super(constants.gyro);
    this.m_constants = constants;
    left = constants.left;
    right = constants.right;
    drive = new DifferentialDrive(left.getBackend(), right.getBackend());
    m_odometry = new DifferentialDriveOdometry(getYaw());

    leftEncoder = constants.leftEncoder;
    rightEncoder = constants.rightEncoder;

    leftEncoder.setPositionConversionFactor(constants.leftEncoderFactor);
    rightEncoder.setPositionConversionFactor(constants.rightEncoderFactor);
    leftEncoder.setVelocityConversionFactor(constants.leftEncoderFactor);
    rightEncoder.setVelocityConversionFactor(constants.rightEncoderFactor);
  }

  /**
   * Periodic function for TankDrive subsystem. Updates the odometry.
   * 
   * @see DifferentialDriveOdometry#update(Rotation2d, double, double)
   */
  @Override
  public void periodic() {
    m_odometry.update(getYaw(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Drives the robot using the arcade drive mode
   * 
   * @param forward  The robot's speed along the x axis [-1, 1]. Forward is
   *                 positive.
   * @param rotation The robot's turning rate around the z axis [-1, 1]. Clockwise
   *                 is positive.
   */
  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  /**
   * Drives the robot using the tank drive mode
   * 
   * @param left  The robot's left side speed along the X axis [-1, 1]. Forward is
   *              positive.
   * @param right The robot's right side speed along the X axis [-1, 1]. Forward
   *              is positive.
   */
  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
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
    m_odometry.resetPosition(pose, getYaw());
  }

  /**
   * Resets the left and right encoder positions to zero
   */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * 
   * @return the speeds of both sides of the robot
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  /**
   * Drives the robot using the tank drive mode with input in volts
   * 
   * @param leftVolts  Voltage to send to left side motors
   * @param rightVolts Voltage to send to right side motors
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drive.feed();
  }

  @Override
  public void rotate(double speed) {
    arcadeDrive(0, speed);
  }

  @Override
  public void stop() {
    arcadeDrive(0, 0);
  }

  @Override
  public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose){
    var feedforward = new SimpleMotorFeedforward(m_constants.ksVolts,
        m_constants.kvVoltSecondsPerMeter,
        m_constants.kaVoltSecondsSquaredPerMeter);
    // Helper class to calculate voltage sent to motors
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            feedforward,
            m_constants.kDriveKinematics,
            10);

    // Create config for trajectory
    
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to correctly calculate wheel speeds
            .setKinematics(m_constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // The trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Starting pose
        startPose,
        // List of waypoints to pass through
        waypoints,
        // Ending pose
        endPose,
        // Pass config to generate Trajectory properly for this drivebase
        config
    );

    return trajectory;
  }

  @Override
  public Command generateTrajectoryCommand(Trajectory trajectory) {
    
    var feedforward = new SimpleMotorFeedforward(m_constants.ksVolts,
        m_constants.kvVoltSecondsPerMeter,
        m_constants.kaVoltSecondsSquaredPerMeter);
  

    return new RamseteCommand(
        // Trajectory to travel
        trajectory,
        // RamseteCommand needs to be able to 
        // access the pose of the robot at any time
        this::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        // Feedforward to ensure correct voltage is sent to motors
        feedforward,
        // Kinematics to calculate correct wheel speeds
        m_constants.kDriveKinematics,
        // RamseteCommand needs t be able to access the speeds of the wheels at any time
        this::getWheelSpeeds,
        // PID Controllers used to compensate for external factors in motor speed
        new PIDController(m_constants.kPDriveVel, 0, 0),
        new PIDController(m_constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback to drive the wheels
        this::tankDriveVolts,
        this
    );
  }

  @Override
  public void driveForward(double forward, double rotate) {
    arcadeDrive(forward, rotate);
  }
}
