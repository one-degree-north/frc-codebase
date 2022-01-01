// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ODN_Drivebase;
import frc.lib.encoder.Encoder;
import frc.lib.motorcontroller.MotorControllerGroup;
import frc.robot.Constants.AutoConstants;

public class TankDriveSubsystem extends SubsystemBase implements ODN_Drivebase {
  public static class Constants {
    // MotorControllers
    public MotorControllerGroup left;
    public MotorControllerGroup right;

    // Encoders
    public Encoder leftEncoder;
    public Encoder rightEncoder;

    // CAN ID for gyro
    public int id_gyro;

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

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private MotorControllerGroup left;
  private MotorControllerGroup right;

  private DifferentialDrive drive;

  public Constants m_constants;

  private PigeonIMU gyro;

  private DifferentialDriveOdometry m_odometry;

  /**
   * Constructor for the TankDrive subsystem.
   * 
   * @param constants Object containing all the constants for this drivebase
   */
  public TankDriveSubsystem(Constants constants) {
    this.m_constants = constants;
    left = constants.left;
    right = constants.right;
    drive = new DifferentialDrive(left.getBackend(), right.getBackend());
    gyro = new PigeonIMU(constants.id_gyro);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));

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
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderPosition(), rightEncoderPosition());
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
   * 
   * @return The distance travelled by the left side of the robot
   */
  public double leftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  /**
   * 
   * @return The distance travelled by the right side of the robot
   */
  public double rightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  /**
   * 
   * @return The velocity of the left side of the robot
   */
  public double leftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  /**
   * 
   * @return The velocity of the right side of the robot
   */
  public double rightEncoderVelocity() {
    return rightEncoder.getVelocity();
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
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
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
    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity(), rightEncoderVelocity());
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
  public Command generateTrajectoryCommand(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
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
