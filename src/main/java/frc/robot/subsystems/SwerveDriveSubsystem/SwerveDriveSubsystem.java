// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.ODN_Drivebase;
import frc.lib.encoder.Encoder;
import frc.lib.motorcontroller.MotorController;
import frc.robot.Constants.AutoConstants;

public class SwerveDriveSubsystem extends SubsystemBase implements ODN_Drivebase {
	public static class Constants {
		// diameter of the wheels
		public double wheelDiameterMeters;
		// maximum speed robot can drive at
		public double maxSpeedMetersPerSecond;

		// is the gyro backwards?
		public boolean gyroReversed;

		// CAN IDs for various CAN devices
		// CAN IDs for drive motors
		public MotorController id_md_fl;
		public MotorController id_md_bl;
		public MotorController id_md_fr;
		public MotorController id_md_br;
		// CAN IDs for turning motors
		public MotorController id_mt_fl;
		public MotorController id_mt_bl;
		public MotorController id_mt_fr;
		public MotorController id_mt_br;
		// CAN IDs for turning encoders
		public Encoder id_et_fl;
		public Encoder id_et_bl;
		public Encoder id_et_fr;
		public Encoder id_et_br;
		// CAN IDs for drive encoders
		public Encoder id_ed_fl;
		public Encoder id_ed_bl;
		public Encoder id_ed_fr;
		public Encoder id_ed_br;

		public SwerveDriveKinematics driveKinematics;

		// Offset for absolute encoders
		public double offset_fl;
		public double offset_bl;
		public double offset_fr;
		public double offset_br;

		// Distance between front and back wheels on robot
		public double wheelBase;
		// Distance between centers of right and left wheels on robot
		public double trackWidth;
	}

	// Robot swerve modules
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_rearLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_rearRight;

	// The gyro sensor
	private final AHRS m_gyro;

	// Odometry class for tracking robot pose
	private final SwerveDriveOdometry m_odometry;

	public final Constants m_constants;

	public SwerveDriveSubsystem(Constants constants) {
		this.m_constants = constants;

		this.m_frontLeft = new SwerveModule(m_constants.id_md_fl, m_constants.id_mt_fl, m_constants.id_ed_fl,
				m_constants.id_et_fl, m_constants.offset_fl, m_constants);
		this.m_rearLeft = new SwerveModule(m_constants.id_md_bl, m_constants.id_mt_bl, m_constants.id_ed_bl,
				m_constants.id_et_bl, m_constants.offset_bl, m_constants);
		this.m_frontRight = new SwerveModule(m_constants.id_md_fr, m_constants.id_mt_fr, m_constants.id_ed_fr,
				m_constants.id_et_fr, m_constants.offset_fr, m_constants);
		this.m_rearRight = new SwerveModule(m_constants.id_md_br, m_constants.id_mt_br, m_constants.id_ed_br,
				m_constants.id_et_br, m_constants.offset_br, m_constants);

		this.m_gyro = new AHRS();
		this.m_odometry = new SwerveDriveOdometry(m_constants.driveKinematics, getRotation2d());
	}

	public double getYaw() {
		return m_gyro.getYaw();
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
		return m_odometry.getPoseMeters().getTranslation();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(getRotation2d(), m_frontLeft.getState(), m_rearLeft.getState(), m_frontRight.getState(),
				m_rearRight.getState());
		System.out.println("chungus: "+getYaw());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	@Override
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
		m_gyro.reset();
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

	@Override
	public void rotate(double speed) {
		drive(0, 0, speed, false);
	}

	@Override
	public void stop() {
		drive(0, 0, 0, false);
	}

	@Override
	public Command generateTrajectoryCommand(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
		// Create config for trajectory
		TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_constants.driveKinematics);
    
    // The trajectory to follow.  All units in meters.
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


    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new SwerveControllerCommand(
        // Trajectory to travel
        trajectory,
        // SwerveControllerCommand needs to be able to 
        // access the pose of the robot at any time
        this::getPose,
        // Kinematics to calculate correct wheel speeds
        m_constants.driveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        // Rotation controller
        thetaController,
        // SwerveControllerCommand passes desired module states to the callback
        this::setModuleStates,
        this
		);
	}
}