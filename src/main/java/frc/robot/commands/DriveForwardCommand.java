// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class DriveForwardCommand extends CommandBase {
  private SwerveDriveSubsystem m_swerve;

  /** Creates a new DriveBackCommand. */
  public DriveForwardCommand(SwerveDriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  Translation2d start;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetYaw();
    m_swerve.resetOdometry(new Pose2d());
    start = m_swerve.getPose().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.cartesianDriveRelative(-0.3, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_swerve.getPose().getTranslation().getDistance(start)) > 50;
  }
}
