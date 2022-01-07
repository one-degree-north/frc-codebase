// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ODN_Drivebase;

public class TrajectoryCommand extends CommandBase {
  /** Creates a new TrajectoryCommand. */

  private ODN_Drivebase m_drive;
  private Command m_command;
  private Pose2d m_startPose;

  /**
   * Constructs a TrajectoryCommand object with position data
   * @param drive Drivebase subsystem to follow the trajectory
   * @param startPose The starting pose of the robot (sets relative coordinate system)
   * @param waypoints The points for the robot to reach before the end position
   * @param endPose The ending pose
   */
  public TrajectoryCommand(ODN_Drivebase drive, Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
    this(drive, drive.generateTrajectory(startPose, waypoints, endPose));
  }

  /**
   * Constructs a TrajectoryCommand object with a trajectory object
   * @param drive Drivebase subsystem to follow the trajectory
   * @param traj The trajectory to follow
   */
  public TrajectoryCommand(ODN_Drivebase drive, String name, double maxVel, double maxAccel) {
    this(drive, PathPlanner.loadPath(name, maxVel, maxAccel));
  }

  /**
   * Constructs a TrajectoryCommand object with a trajectory object (NOT ACCESSIBLE)
   * @param drive Drivebase subsystem to follow the trajectory
   * @param traj The trajectory to follow
   */
  private TrajectoryCommand(ODN_Drivebase drive, Trajectory traj) {
    this.m_drive = drive;

    m_command = drive.generateTrajectoryCommand(traj);
    m_startPose = traj.getInitialPose();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_startPose);

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
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
