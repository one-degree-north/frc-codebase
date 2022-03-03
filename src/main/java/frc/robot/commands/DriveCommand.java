// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

public class DriveCommand extends CommandBase {
  private SwerveDriveSubsystem m_swerve;
  private double m_direction;
  private double m_distance;
  /** Creates a new DriveBackCommand. */
  public DriveCommand(SwerveDriveSubsystem swerve, double direction, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_direction = direction;
    m_distance = distance;
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
    Rotation2d direction = Rotation2d.fromDegrees(m_direction);
    double d = m_swerve.getYaw().getRadians();
    if(d > Math.PI) d -= 2 * Math.PI;
    
    m_swerve.cartesianDriveRelative(0.1*direction.getSin(), -0.1*direction.getCos(), -d*2);
    System.out.println((m_swerve.getPose().getTranslation().getDistance(start)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_swerve.getPose().getTranslation().getDistance(start)) > m_distance;
  }
}