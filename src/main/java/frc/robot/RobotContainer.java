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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.FalconMusicSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here: 
  private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);
  public ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants);
  public ShooterSubsystem m_shoot = new ShooterSubsystem(Constants.shootConstants);

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
      m_drive.setDefaultCommand(new RunCommand(() -> {

       m_drive.cartesianDriveRelative(modifyAxis(m_controller.getLeftY()),
       modifyAxis(m_controller.getLeftX()),
       modifyAxis(m_controller.getRightX()));
       },
       m_drive)); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton climbUp = new JoystickButton(m_controller, XboxController.Button.kA.value);
    JoystickButton climbDown = new JoystickButton(m_controller, XboxController.Button.kY.value);
    climbUp.whenPressed(
      new InstantCommand(()->m_climb.set(0.5), m_climb)
      );
    climbDown.whenPressed(
      new InstantCommand(()->m_climb.set(-0.5), m_climb)
      );
    climbUp.whenReleased(
      new InstantCommand(() -> m_climb.set(0), m_climb)
    );
    climbDown.whenReleased(
      new InstantCommand(() -> m_climb.set(0), m_climb)
    );
    JoystickButton highShoot = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    JoystickButton lowShoot = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    Trigger intake = new Trigger(()->m_controller.getRightTriggerAxis() > 0.7);
    highShoot.whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(()->m_shoot.setShoot(0.8), m_shoot),
        new WaitCommand(0.5),
        new InstantCommand(()->m_shoot.setIndex(0.7),m_shoot)
      )
    );
    highShoot.whenReleased(
      new InstantCommand(()->m_shoot.disableAll(), m_shoot)
    );
    lowShoot.whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(()->m_shoot.setShoot(0.2), m_shoot),
        new WaitCommand(0.5),
        new InstantCommand(()->m_shoot.setIndex(0.7),m_shoot)
      )
    );
    lowShoot.whenReleased(
      new InstantCommand(()->m_shoot.disableAll(), m_shoot)
    );
    intake.whenActive(new InstantCommand(()-> m_shoot.setIntake(0.7), m_shoot));
    intake.whenInactive(new InstantCommand(()-> m_shoot.setIntake(0), m_shoot));
    
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
