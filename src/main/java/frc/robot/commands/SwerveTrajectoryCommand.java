// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDriveSubsystem.SwerveDriveSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {
  /** Creates a new TrajectoryCommand. */

  private SwerveDriveSubsystem m_drive;
  private Trajectory m_trajectory;
  private SwerveControllerCommand m_command;

  /**
   * Constructs a TrajectoryCommand object
   * @param drive TankDrive subsystem to drive the trajectory
   * @param startPose The starting pose of the robot (sets relative coordinate system)
   * @param waypoints The points for the robot to reach before the end position
   * @param endPose The ending pose (relate to the starting pose)
   */
  public SwerveTrajectoryCommand(SwerveDriveSubsystem m_robotDrive, List<Translation2d> waypoint, Pose2d end) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_robotDrive.m_constants.driveKinematics);

    m_drive=m_robotDrive;
    
    // An example trajectory to follow.  All units in meters.
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            waypoint,
            // End 3 meters straight ahead of where we started, facing forward
            end,
            config);


    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_command =
        new SwerveControllerCommand(
            m_trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            m_drive.m_constants.driveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    addRequirements(m_drive);
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
    m_drive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
