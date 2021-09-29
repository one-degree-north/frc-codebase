// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {
  public static class Constants {
    public int id_fl;
	  public int id_bl;
	  public int id_fr;
    public int id_br;
    public int id_gyro;
    public double leftEncoderFactor;
    public double rightEncoderFactor;

    public double ksVolts;
    public double kvVoltSecondsPerMeter;
    public double kaVoltSecondsSquaredPerMeter;
    public DifferentialDriveKinematics kDriveKinematics;
	  public double kPDriveVel;
  }

  private WPI_TalonSRX frontLeft;
  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX backLeft;
  private WPI_TalonSRX backRight;

  private SpeedControllerGroup left;
  private SpeedControllerGroup right;

  private DifferentialDrive drive;

  public Constants m_constants;

  private PigeonIMU gyro;

  private DifferentialDriveOdometry m_odometry;

  /** Creates a new TankDrive. */
  public TankDrive(Constants constants) {
    this.m_constants = constants;
    frontLeft = new WPI_TalonSRX(constants.id_fl);
    frontRight = new WPI_TalonSRX(constants.id_fr);
    backLeft = new WPI_TalonSRX(constants.id_bl);
    backRight = new WPI_TalonSRX(constants.id_br);
    left = new SpeedControllerGroup(frontLeft, backLeft);
    right = new SpeedControllerGroup(frontRight, backRight);
    drive = new DifferentialDrive(left, right);
    gyro = new PigeonIMU(constants.id_gyro);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderPosition(),
                      rightEncoderPosition());
  }

  public double getYaw() {
    double[] arr = new double[3];
    gyro.getYawPitchRoll(arr);
    return arr[0];
  }

  public double leftEncoderPosition() {
    return frontLeft.getSelectedSensorPosition(0) * m_constants.leftEncoderFactor;
  }

  public double rightEncoderPosition() {
    return frontRight.getSelectedSensorPosition(0) * m_constants.rightEncoderFactor;
  }

  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }




  public double leftEncoderVelocity() {
    return frontLeft.getSelectedSensorVelocity(0) * m_constants.leftEncoderFactor;
  }

  public double rightEncoderVelocity() {
    return frontRight.getSelectedSensorVelocity(0) * m_constants.rightEncoderFactor;
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
  }

  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity(), rightEncoderVelocity());
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
    drive.feed();
  }
}
