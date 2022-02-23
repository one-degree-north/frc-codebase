// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;

public class IndexerCommand extends CommandBase {
  private MotorControllerSubsystem m_intake;
  private boolean isIntaking;
  /** Creates a new IndexerCommand. */
  public IndexerCommand(MotorControllerSubsystem intake, boolean intaking) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    isIntaking = intaking;
    addRequirements(m_intake);
  }

  double t_s;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isIntaking){
      m_intake.set(0.5);
    }
    else{
      m_intake.set(-0.5);

    }
    t_s = System.currentTimeMillis()/1000.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis()/1000.0-t_s > 0.1){
      if(isIntaking){
        m_intake.set(0.5);
      }
      else{
        m_intake.set(-0.4);

      }
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis()/1000.0-t_s > 1);
  }
}
