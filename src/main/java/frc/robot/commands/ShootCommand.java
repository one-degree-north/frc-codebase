// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
  }

  double start;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.on();
    this.indexer.onshoot();
    start = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.off();
    this.indexer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()-start)/1000 > 1;
  }
}
