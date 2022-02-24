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
  private boolean shootHigh;

  
  public ShooterCommand(MotorControllerSubsystem top,  MotorControllerSubsystem bottom, Function<Double, Double> shooter, boolean high) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_top = top;
    m_bottom = bottom;
    m_shooter = shooter;
    shootHigh = high;
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
    if(shootHigh){
      m_top.setSpeed(2500);
      m_bottom.setSpeed(2500);
      // m_top.setSpeed(m_shooter.apply(RobotContainer.container.getAngle()));
      // m_bottom.setSpeed(m_shooter.apply(RobotContainer.container.getAngle()));
    }
    else{
      m_top.setSpeed(1500);
      m_bottom.setSpeed(1500);
      System.out.println("on");
    }



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
    return false;
    //return System.currentTimeMillis()/1000.0-t_s>3;
  }
}
