// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.lib.basesubsystem.MotorControllerSubsystem;

public class ShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  private MotorControllerSubsystem m_top;
  private MotorControllerSubsystem m_bottom;
  private Function<Double, Double> m_shooter;

  
  public ShooterCommand(MotorControllerSubsystem top,  MotorControllerSubsystem bottom, Function<Double, Double> shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_top = top;
    m_bottom = bottom;
    m_shooter = shooter;
    addRequirements(m_top, m_bottom);

  }


  double t_s;
  boolean done = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    t_s = System.currentTimeMillis()/1000.0;
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis()/1000.0-t_s>0.6){
      done = true;
    }
    else if(System.currentTimeMillis()/1000.0-t_s>0.1 && System.currentTimeMillis()/1000.0-t_s<0.6){
        m_top.setSpeed(m_shooter.apply(RobotContainer.container.getAngle()+33));
    }
    
    m_bottom.setSpeed(0.75*m_shooter.apply(RobotContainer.container.getAngle()+33));



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_bottom.setSpeed(0);
    m_top.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
