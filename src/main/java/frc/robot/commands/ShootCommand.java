// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.ODN_HolonomicDrivebase;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.robot.subsystems.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  //offset is 25 degrees, min is 55, max is 70, height is 107.95
  public static Function<Double, Double> hood(double min, double max, double offset, double height) {
    return (x)->{
      x+=offset;
      x = height/Math.tan(x);
      x = -0.00002514*x*x-0.00001541*x+70.82;
      if(x<min){
        x=min;
      }
      if(x>max){
        x=max;
      }
      return x;
    };
  }

  //ft/s conversion: 60/(radius (ft) *2 *Math.Pi * 6380)
  public static Function<Double, Double> shoot(double offset, double height, double min, double max, double conversion) {
    return (x)->{
      x+=offset;
      x = height/Math.tan(x);
      x = -0.00002591*x*x+0.0394*x+16.06;
      if(x<min){
        x=min;
      }
      if(x>max){
        x=max;
      }
      return x*conversion;
      
    };
  }
;
  public ShootCommand(ODN_HolonomicDrivebase drive, LimelightSubsystem m_limelight, MotorControllerSubsystem m_intake, MotorControllerSubsystem m_shooterTop, MotorControllerSubsystem m_shooterBottom,  XboxController stick){
    double conversion = 60/(1/3 *2 *Math.PI * 6380);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignCommand(drive, m_limelight, LimelightSubsystem.linearAttenuation(27), hood(55.0, 70.0, 25.0, 107.95), stick),
      new IndexerCommand(m_intake, true),
      new ShooterCommand(m_shooterTop, m_shooterBottom, shoot(25, 107.95, 21, 30, conversion), true)

    );
  }
}
