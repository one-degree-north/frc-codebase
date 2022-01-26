// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Joystick;
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

  public static Function<Double, Double> hood(double min, double max, double offset, double height) {
    return (x)->{return 0.0;};
  }

  public static Function<Double, Double> shoot(double offset, double height) {
    return (x)->{return 0.0;};
  }
  
  public ShootCommand(ODN_HolonomicDrivebase drive, LimelightSubsystem m_limelight, MotorControllerSubsystem m_intakeFront,
   MotorControllerSubsystem m_intakeBack, MotorControllerSubsystem m_shooterTop, MotorControllerSubsystem m_shooterBottom, HoodSubsystem m_hood, Joystick stick){

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new AlignCommand(drive, m_limelight, m_hood, LimelightSubsystem.linearAttenuation(27), stick));
  }
}
