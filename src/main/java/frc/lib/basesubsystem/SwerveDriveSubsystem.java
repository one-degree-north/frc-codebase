// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import java.util.List;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

		public int FRONT_LEFT_MODULE_DRIVE_MOTOR; // FIXME Set front left module drive motor ID
		public int FRONT_LEFT_MODULE_STEER_MOTOR; // FIXME Set front left module steer motor ID
		public int FRONT_LEFT_MODULE_STEER_ENCODER; // FIXME Set front left steer encoder ID
		public double FRONT_LEFT_MODULE_STEER_OFFSET; // FIXME Measure and set front left steer offset

		public int FRONT_RIGHT_MODULE_DRIVE_MOTOR; // FIXME Set front right drive motor ID
		public int FRONT_RIGHT_MODULE_STEER_MOTOR; // FIXME Set front right steer motor ID
		public int FRONT_RIGHT_MODULE_STEER_ENCODER; // FIXME Set front right steer encoder ID
		public double FRONT_RIGHT_MODULE_STEER_OFFSET; // FIXME Measure and set front right steer offset

		public int BACK_LEFT_MODULE_DRIVE_MOTOR; // FIXME Set back left drive motor ID
		public int BACK_LEFT_MODULE_STEER_MOTOR; // FIXME Set back left steer motor ID
		public int BACK_LEFT_MODULE_STEER_ENCODER; // FIXME Set back left steer encoder ID
		public double BACK_LEFT_MODULE_STEER_OFFSET; // FIXME Measure and set back left steer offset

		public int BACK_RIGHT_MODULE_DRIVE_MOTOR; // FIXME Set back right drive motor ID
		public int BACK_RIGHT_MODULE_STEER_MOTOR; // FIXME Set back right steer motor ID
		public int BACK_RIGHT_MODULE_STEER_ENCODER; // FIXME Set back right steer encoder ID
		public double BACK_RIGHT_MODULE_STEER_OFFSET; // FIXME Measure and set back right steer offset

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
	public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0
			/ SdsModuleConfigurations.MK3_STANDARD.getDriveReduction()
			* SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
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

	private boolean isLocked = false;

	public SwerveDriveSubsystem(Constants constants) {
		super(constants.gyro);

		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

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

		// There are 4 methods you can call to create your swerve modules.
		// The method you use depends on what motors you are using.
		//
		// Mk3SwerveModuleHelper.createFalcon500(...)
		// Your module has two Falcon 500s on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createNeo(...)
		// Your module has two NEOs on it. One for steering and one for driving.
		//
		// Mk3SwerveModuleHelper.createFalcon500Neo(...)
		// Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
		// and the NEO is for steering.
		//
		// Mk3SwerveModuleHelper.createNeoFalcon500(...)
		// Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
		// Falcon 500 is for steering.
		//
		// Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
		// class.

		// By default we will use Falcon 500s in standard configuration. But if you use
		// a different configuration or motors
		// you MUST change it. If you do not, your code will crash on startup.
		// FIXME Setup motor configuration

		m_frontLeftModule = Mk3SwerveModuleHelper.createNeo(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
				// This can either be STANDARD or F
				// AST depending on your gear configuration
				Mk3SwerveModuleHelper.GearRatio.STANDARD,
				// This is the ID of the drive motor
				constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
				// This is the ID of the steer motor
				constants.FRONT_LEFT_MODULE_STEER_MOTOR,
				// This is the ID of the steer encoder
				constants.FRONT_LEFT_MODULE_STEER_ENCODER,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				constants.FRONT_LEFT_MODULE_STEER_OFFSET);

		// We will do the same for the other modules
		m_frontRightModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD, constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				constants.FRONT_RIGHT_MODULE_STEER_MOTOR, constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_backLeftModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD, constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				constants.BACK_LEFT_MODULE_STEER_MOTOR, constants.BACK_LEFT_MODULE_STEER_ENCODER,
				constants.BACK_LEFT_MODULE_STEER_OFFSET);

		m_backRightModule = Mk3SwerveModuleHelper.createNeo(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
				Mk3SwerveModuleHelper.GearRatio.STANDARD, constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				constants.BACK_RIGHT_MODULE_STEER_MOTOR, constants.BACK_RIGHT_MODULE_STEER_ENCODER,
				constants.BACK_RIGHT_MODULE_STEER_OFFSET);
	}

	@Override
	public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate) {
	if(!isLocked){
		ChassisSpeeds m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * MAX_VELOCITY_METERS_PER_SECOND,
				ySpeed * MAX_VELOCITY_METERS_PER_SECOND, rotate * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
				getYaw());
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
	}
	}

	@Override
	public void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate) {
	if(!isLocked){
		ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(xSpeed * MAX_VELOCITY_METERS_PER_SECOND,
				ySpeed * MAX_VELOCITY_METERS_PER_SECOND, rotate * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		setModuleStates(states);
		}
	}

	@Override
	public void polarDrive(double speed, Rotation2d direction, double rotate) {
	if(!isLocked){
		cartesianDriveAbsolute(speed * -direction.getSin(), speed * direction.getCos(),
				rotate);
		}
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

	public void setLock(boolean lock){
		isLocked = lock;
	}

	public void setLocke(int stupidity){
		if(stupidity == 69)
			System.out.println("god is dead");
		else 
			System.out.println("hello vayun");
	}
	
}