// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.lib.ODN_HolonomicDrivebase;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.gyro.ODN_Gyro;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.robot.Constants.AutoConstants;

public class MecanumDriveSubsystem extends ODN_HolonomicDrivebase {
  public static class Constants {
    // ODN_MotorControllers
    public ODN_MotorController frontLeft;
    public ODN_MotorController rearLeft;
    public ODN_MotorController frontRight;
    public ODN_MotorController rearRight;

    // Encoders
    public ODN_Encoder frontLeftEncoder;
    public ODN_Encoder rearLeftEncoder;
    public ODN_Encoder frontRightEncoder;
    public ODN_Encoder rearRightEncoder;

    // gyro
    public ODN_Gyro gyro;

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

  private ODN_Encoder frontLeftEncoder;
  private ODN_Encoder rearLeftEncoder;
  private ODN_Encoder frontRightEncoder;
  private ODN_Encoder rearRightEncoder;

  private ODN_MotorController frontLeft;
  private ODN_MotorController rearLeft;
  private ODN_MotorController frontRight;
  private ODN_MotorController rearRight;

  private MecanumDrive drive;

  public Constants m_constants;

  private MecanumDriveOdometry m_odometry;

  /**
   * Constructor for the TankDrive subsystem.
   * 
   * @param constants Object containing all the constants for this drivebase
   */
  public MecanumDriveSubsystem(Constants constants) {
    super(constants.gyro);
    this.m_constants = constants;
    frontLeft = constants.frontLeft;
    rearLeft = constants.rearLeft;
    frontRight = constants.frontRight;
    rearRight = constants.rearRight;
    drive = new MecanumDrive(frontLeft.getBackend(), rearLeft.getBackend(), frontRight.getBackend(), rearRight.getBackend());
    m_odometry = new MecanumDriveOdometry(constants.kDriveKinematics, getYaw());

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
    m_odometry.update(getYaw(), new MecanumDriveWheelSpeeds(frontLeftEncoder.getVelocity(),
        frontRightEncoder.getVelocity(), rearLeftEncoder.getVelocity(), rearRightEncoder.getVelocity()));
  }

  public void cartesianDriveRelative(double xSpeed, double ySpeed, double zRotation) {
    drive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double zRotation) {
    drive.driveCartesian(ySpeed, xSpeed, zRotation, getYaw().getDegrees());
  }

  public void polarDrive(double magnitude, Rotation2d direction, double zRotation) {
    drive.drivePolar(magnitude, direction.getDegrees(), zRotation);
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
  public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose){
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

    return trajectory;
  }
  
  @Override
  public Command generateTrajectoryCommand(Trajectory trajectory) {
    var feedforward = new SimpleMotorFeedforward(m_constants.ksVolts, m_constants.kvVoltSecondsPerMeter,
        m_constants.kaVoltSecondsSquaredPerMeter);

    // Create config for trajectory

        var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, new TrapezoidProfile.Constraints(
              AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
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
