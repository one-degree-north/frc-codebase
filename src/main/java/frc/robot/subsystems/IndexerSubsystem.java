// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.sensor.ODN_ColorSensor;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootAwayCommand;
import frc.robot.commands.ShootTowardCommand;

public class IndexerSubsystem extends SubsystemBase {
  private static final Color RED = new Color(1, 0, 0);
  private static final Color BLUE = new Color(0, 0, 1);

  private static final ColorMatch m_matcher = ODN_ColorSensor.createMatcher(0.8, RED, BLUE);

  public static class Constants {
    public MotorControllerSubsystem motor;
    public ODN_ColorSensor color;
    public DigitalInput breakbeam_enter;
    public DigitalInput breakbeam_exit;
  }
  private MotorControllerSubsystem m_motor;
  private ODN_ColorSensor m_color;
  public DigitalInput m_breakbeam_enter;
  public DigitalInput m_breakbeam_exit;


  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(Constants constants) {
    m_motor = constants.motor;
    m_color = constants.color;
    m_breakbeam_enter = constants.breakbeam_enter;
    m_breakbeam_exit = constants.breakbeam_exit;
  }

  @Override
  public void periodic() {
    Color match_res = m_matcher.matchClosestColor(m_color.getColor()).color;
    if(match_res.equals(RED)) {
      new ShootAwayCommand(RobotContainer.container.getIndexer(), 
      RobotContainer.container.getDrivebase(), 
      RobotContainer.container.getShooter()).schedule();
    } else if(match_res.equals(BLUE)) {
      new ShootTowardCommand(RobotContainer.container.getIndexer(), 
      RobotContainer.container.getDrivebase(), 
      RobotContainer.container.getShooter(), 
      RobotContainer.container.getLimelight(),
      RobotContainer.container.getJoystick()).schedule();
    }
    if(m_breakbeam_exit.get())
    {
      RobotContainer.container.getIndexer().off();
    }
    if(m_breakbeam_enter.get())
    {
      RobotContainer.container.getIndexer().on();
    }

    // This method will be called once per scheduler run
  }

  public void on() {
    m_motor.set(0.8);
  }

  public void off() {
    m_motor.set(0);
  }
}
