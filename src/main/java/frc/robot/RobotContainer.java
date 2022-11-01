// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.FalconMusicSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here: 
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private PneumaticSubsystem m_piston = new PneumaticSubsystem(Constants.pneumaticConstants);
  // Controllers here:
  private XboxController m_controller = new XboxController(0);
  private Compressor m_compressor = new Compressor(10, PneumaticsModuleType.REVPH);

  // Robot commands go here:
  public Command nullCommand() {
    return null;
  }
  
  // This command runs on autonomous
  // private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands here; template for swerve is below
    //   m_drive.setDefaultCommand(new RunCommand(() -> {

    //    m_drive.cartesianDriveRelative(modifyAxis(m_controller.getLeftY()),
    //    modifyAxis(m_controller.getLeftX()),
    //    modifyAxis(m_controller.getRightX()));
    //    },
    //    m_drive)); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton retract = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    JoystickButton extend = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    JoystickButton toggleCompressor = new JoystickButton(m_controller, XboxController.Button.kB.value);

    retract.whenActive(new InstantCommand(()->m_piston.set(Value.kForward), m_piston));
    extend.whenPressed(new InstantCommand(()->m_piston.set(Value.kReverse), m_piston));
    toggleCompressor.whenPressed(new ConditionalCommand(
      new InstantCommand(() -> m_compressor.disable()), // True Command
      new InstantCommand(() -> m_compressor.enableAnalog(90, 120)), // False Command
      () -> m_compressor.enabled() // Conditional (BooleanSupplier)
      )); 
  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}

// Agastya