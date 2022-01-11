// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.gyro.ODN_Gyro;
import frc.lib.ODN_HolonomicDrivebase;
import frc.robot.Constants.AutoConstants;

public class SwerveDriveSubsystem extends ODN_HolonomicDrivebase {

	public static class Constants {

		public double DRIVETRAIN_TRACKWIDTH_METERS = 0.47; // FIXME Measure and set trackwidth
		public double DRIVETRAIN_WHEELBASE_METERS; // FIXME Measure and set wheelbase

		public SwerveModule frontLeftModule;
		public SwerveModule frontRightModule;
		public SwerveModule backLeftModule;
		public SwerveModule backRightModule;
		public ModuleConfiguration config;

		public ODN_Gyro gyro;
	}

	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */

	public static final double MAX_VOLTAGE = 12.0;

	// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
	// SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	public static double maxVelocity(ModuleConfiguration moduleConfig) {
		return 5880.0 / 60.0
		/ moduleConfig.getDriveReduction()
		* moduleConfig.getWheelDiameter() * Math.PI;
	}
	public double MAX_VELOCITY_METERS_PER_SECOND;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

	private final SwerveDriveKinematics m_kinematics;

	private final SwerveDriveOdometry m_odometry;

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	public SwerveDriveSubsystem(Constants constants) {
		super(constants.gyro);

		MAX_VELOCITY_METERS_PER_SECOND = maxVelocity(constants.config);

		MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
				/ Math.hypot(constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
				/ 5;

		m_kinematics = new SwerveDriveKinematics(
				// Front left
				new Translation2d(constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

		this.m_odometry = new SwerveDriveOdometry(m_kinematics, getYaw());

		m_frontLeftModule = constants.frontLeftModule;
		m_frontRightModule = constants.frontRightModule;
		m_backLeftModule = constants.backLeftModule;
		m_backRightModule = constants.backRightModule;
	}

	@Override
	public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate) {

		ChassisSpeeds m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * MAX_VELOCITY_METERS_PER_SECOND,
				ySpeed * MAX_VELOCITY_METERS_PER_SECOND, rotate * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
				getYaw());
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
	}

	@Override
	public void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate) {

		ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(xSpeed * MAX_VELOCITY_METERS_PER_SECOND,
				ySpeed * MAX_VELOCITY_METERS_PER_SECOND, rotate * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
	}

	@Override
	public void polarDrive(double speed, Rotation2d direction, double rotate) {
		cartesianDriveAbsolute(speed * -direction.getSin(), speed * direction.getCos(),
				rotate);
	}

	@Override
	public void periodic() {
		m_odometry.update(getYaw(),
				new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
						new Rotation2d(m_frontLeftModule.getSteerAngle())),
				new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
						new Rotation2d(m_backLeftModule.getSteerAngle())),
				new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
						new Rotation2d(m_frontRightModule.getSteerAngle())),
				new SwerveModuleState(m_backRightModule.getDriveVelocity(),
						new Rotation2d(m_backRightModule.getSteerAngle())));
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void setModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());
	}

	@Override
	public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose){
		TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(m_kinematics);

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
		// Create config for trajectory
		

		var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
		new TrapezoidProfile.Constraints(
			AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		return new SwerveControllerCommand(
				// Trajectory to travel
				trajectory,
				// SwerveControllerCommand needs to be able to
				// access the pose of the robot at any time
				this::getPose,
				// Kinematics to calculate correct wheel speeds
				m_kinematics,
				// Position controllers
				new PIDController(AutoConstants.kPXController, 0, 0),
				new PIDController(AutoConstants.kPYController, 0, 0),
				// Rotation controller
				thetaController,
				// SwerveControllerCommand passes desired module states to the callback
				this::setModuleStates, this);
	}

	@Override
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(pose, getYaw());
	}

	
}