// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
	public static class Constants {
		// diameter of the wheels
		public double wheelDiameterMeters;
		// maximum speed robot can drive at
		public double maxSpeedMetersPerSecond;

		// is the gyro backwards?
		public boolean gyroReversed;

		// CAN IDs for various CAN devices
		// CAN IDs for drive motors
		public int id_md_fl;
		public int id_md_bl;
		public int id_md_fr;
		public int id_md_br;
		// CAN IDs for turning motors
		public int id_mt_fl;
		public int id_mt_bl;
		public int id_mt_fr;
		public int id_mt_br;
		// CAN IDs for encoders
		public int id_e_fl;
		public int id_e_bl;
		public int id_e_fr;
		public int id_e_br;
		// CAN ID for gyro
		public int id_gyro;

		public SwerveDriveKinematics driveKinematics;

		// Offset for absolute encoders
		public double offset_fl;
		public double offset_bl;
		public double offset_fr;
		public double offset_br;
	}

	// Robot swerve modules
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_rearLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_rearRight;

	// The gyro sensor
	private final PigeonIMU m_gyro;

	// Odometry class for tracking robot pose
	private final SwerveDriveOdometry m_odometry;

	public final Constants m_constants;

	public SwerveDriveSubsystem(Constants constants) {
		this.m_constants = constants;

		this.m_frontLeft = new SwerveModule(m_constants.id_md_fl, m_constants.id_mt_fl, 
				m_constants.id_e_fl, m_constants.offset_fl, m_constants);
		this.m_rearLeft = new SwerveModule(m_constants.id_md_bl, m_constants.id_mt_bl, 
				m_constants.id_e_bl, m_constants.offset_bl, m_constants);
		this.m_frontRight = new SwerveModule(m_constants.id_md_fr, m_constants.id_mt_fr, 
				m_constants.id_e_fr, m_constants.offset_fr, m_constants);
		this.m_rearRight = new SwerveModule(m_constants.id_md_br, m_constants.id_mt_br, 
				m_constants.id_e_br, m_constants.offset_br, m_constants);

		this.m_gyro = new PigeonIMU(m_constants.id_gyro);
		this.m_odometry = new SwerveDriveOdometry(m_constants.driveKinematics, getRotation2d());
	}

	public double getYaw() {
		double[] pidge = new double[3];
		m_gyro.getYawPitchRoll(pidge);
		return pidge[0];
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getYaw());
	}

	public void resetAllEncoders() {
		m_frontLeft.resetEncoders(-20.83);
		m_rearLeft.resetEncoders(72.510);
		m_frontRight.resetEncoders(95.098);
		m_rearRight.resetEncoders(48.076);

	}

	public Translation2d getTranslations() {
		return m_odometry.getPoseMeters().getTranslation().times(2.54 / 100);
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(getRotation2d(), m_frontLeft.getState(), m_rearLeft.getState(), m_frontRight.getState(),
				m_rearRight.getState());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(pose, getRotation2d());
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_constants.driveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, m_constants.maxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, m_constants.maxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.setYaw(0);
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return getYaw() * (m_constants.gyroReversed ? -1.0 : 1.0);
	}
}