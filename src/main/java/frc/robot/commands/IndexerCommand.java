// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;

public class IndexerCommand extends CommandBase {
  private MotorControllerSubsystem indexerR;
  private MotorControllerSubsystem indexerL;

  /** Creates a new IndexerCommand. */
  public IndexerCommand( MotorControllerSubsystem right,  MotorControllerSubsystem left) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexerR = right;
    indexerL = left;
    addRequirements(indexerR, indexerL);
  }

  double t_s;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerR.setSpeed(4000);
    indexerL.setSpeed(4000);
    t_s = System.currentTimeMillis()/1000.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerR.setSpeed(0);
    indexerL.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()/1000.0-t_s == 1);
  }
}
