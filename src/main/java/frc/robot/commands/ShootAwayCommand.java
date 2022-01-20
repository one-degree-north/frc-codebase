// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

class TurnCommand extends CommandBase {
  private IndexerSubsystem m_indexer;
  private SwerveDriveSubsystem m_swerve;
  private ShooterSubsystem m_shooter;
  public TurnCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, ShooterSubsystem shooter) {
    m_indexer = indexer;
    m_swerve = swerve;
    m_shooter = shooter;
    addRequirements(m_indexer, m_swerve, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: turn to face back if robot would face hub if forward
    m_swerve.rotate(-m_swerve.getYaw().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_swerve.getYaw().getRadians()) < 0.1;
  }
}

class ShootCommand extends CommandBase {
  private IndexerSubsystem m_indexer;
  private SwerveDriveSubsystem m_swerve;
  private ShooterSubsystem m_shooter;

  private double start_time;

  public ShootCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, ShooterSubsystem shooter) {
    m_indexer = indexer;
    m_swerve = swerve;
    m_shooter = shooter;
    addRequirements(m_indexer, m_swerve, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_time = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.on();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()-start_time) > 3;
  }
}

public class ShootAwayCommand extends SequentialCommandGroup {
  /** Creates a new ShootAwayCommand. */
  public ShootAwayCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, ShooterSubsystem shooter) {
    addCommands(new TurnCommand(indexer, swerve, shooter), new ShootCommand(indexer, swerve, shooter));
  }
}