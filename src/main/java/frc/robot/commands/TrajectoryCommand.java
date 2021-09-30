// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.TankDrive;

public class TrajectoryCommand extends CommandBase {
  /** Creates a new TrajectoryCommand. */

  private TankDrive m_drive;
  private Trajectory m_trajectory;
  private RamseteCommand m_command;

  /**
   * Constructs a TrajectoryCommand object
   * @param drive TankDrive subsystem to drive the trajectory
   * @param startPose The starting pose of the robot (sets relative coordinate system)
   * @param waypoints The points for the robot to reach before the end position
   * @param endPose The ending pose (relate to the starting pose)
   */
  public TrajectoryCommand(TankDrive drive, Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
    this.m_drive = drive;

    // Feedforward to ensure correct voltage is sent to motors
    var feedforward = new SimpleMotorFeedforward(m_drive.m_constants.ksVolts,
        m_drive.m_constants.kvVoltSecondsPerMeter,
        m_drive.m_constants.kaVoltSecondsSquaredPerMeter);
    
    // Helper class to calculate voltage sent to motors
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            feedforward,
            m_drive.m_constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to correctly calculate wheel speeds
            .setKinematics(m_drive.m_constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // The trajectory to follow. All units in meters.
    m_trajectory = TrajectoryGenerator.generateTrajectory(
        // Starting pose
        startPose,
        // List of waypoints to pass through
        waypoints,
        // Ending pose
        endPose,
        // Pass config to generate Trajectory properly for this drivebase
        config
    );

    m_command = new RamseteCommand(
        // Trajectory to travel
        m_trajectory,
        // RamseteCommand needs t be able to access the pose of the robot at any time
        m_drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        // Feedforward to ensure correct voltage is sent to motors
        feedforward,
        // Kinematics to calculate correct wheel speeds
        m_drive.m_constants.kDriveKinematics,
        // RamseteCommand needs t be able to access the speeds of the wheels at any time
        m_drive::getWheelSpeeds,
        // PID Controllers used to compensate for external factors in motor speed
        new PIDController(m_drive.m_constants.kPDriveVel, 0, 0),
        new PIDController(m_drive.m_constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback to drive the wheels
        m_drive::tankDriveVolts,
        m_drive
    );

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_trajectory.getInitialPose());

    m_command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
