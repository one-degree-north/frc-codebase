// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

class TurnToLimelightCommand extends CommandBase {
  private IndexerSubsystem m_indexer;
  private SwerveDriveSubsystem m_swerve;
  private ShooterSubsystem m_shooter;
  private LimelightSubsystem m_limelight;
  public TurnToLimelightCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, ShooterSubsystem shooter, LimelightSubsystem limelight) {
    m_indexer = indexer;
    m_swerve = swerve;
    m_shooter = shooter;
    m_limelight = limelight;
    addRequirements(m_indexer, m_swerve, m_shooter, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.rotate(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.foundTarget();
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

public class ShootTowardCommand extends SequentialCommandGroup {
  /** Creates a new ShootAwayCommand. */
  public ShootTowardCommand(IndexerSubsystem indexer, SwerveDriveSubsystem swerve, ShooterSubsystem shooter, LimelightSubsystem limelight, XboxController joystick) {
    addCommands(new TurnToLimelightCommand(indexer, swerve, shooter, limelight), new MoveToSetDistance(swerve, limelight, joystick, Constants.DIST_TO_SHOOT_FROM), new ShootCommand(indexer, swerve, shooter));
  }
}
