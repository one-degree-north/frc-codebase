// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LimelightArcCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MotorControllerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.TrajectoryCommand;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_TalonFX;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:

  //Drivebase
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);

  //Limelight
  private LimelightSubsystem m_limelight = new LimelightSubsystem();

  //Intake and Indexer
  private MotorControllerSubsystem m_intakeFront = new MotorControllerSubsystem(Constants.intakeFrontConstants);
  private MotorControllerSubsystem m_intakeBack = new MotorControllerSubsystem(Constants.intakeBackConstants);

  //Shooter
  private MotorControllerSubsystem m_shooterTop = new MotorControllerSubsystem(Constants.shooterTopConstants);
  private MotorControllerSubsystem m_shooterBottom = new MotorControllerSubsystem(Constants.shooterBottomConstants);
  private MotorControllerSubsystem m_hood = new MotorControllerSubsystem(Constants.hoodConstants);

  //Climber
  private MotorControllerSubsystem m_climberRotate = new MotorControllerSubsystem(Constants.climberRotateConstants);
  private MotorControllerSubsystem m_climberReach = new MotorControllerSubsystem(Constants.climberReachConstants);


  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_drive.setDefaultCommand(new RunCommand(() -> {
    //     m_drive.cartesianDriveAbsolute(modifyAxis(m_controller.getLeftY()), 
    //       modifyAxis(m_controller.getLeftX()),
    //       modifyAxis(m_controller.getRightX()));
    //     m_shooter.setSpeed(100);
    //     System.out.println(m_shooter.getSpeed());
    //   },
    //   m_drive));

    
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton button = new JoystickButton(m_controller, XboxController.Button.kA.value);
    // button.toggleWhenPressed(new LimelightArcCommand(m_drive, m_limelight, LimelightSubsystem.linearAttenuation(27), m_controller));
    // JoystickButton button2 = new JoystickButton(m_controller, XboxController.Button.kB.value);
    // button2.whenPressed(new InstantCommand(()->m_drive.resetYaw(), m_drive));

    //Intake and Indexer
    JoystickButton testBtn = new JoystickButton(m_controller, XboxController.Button.kX.value);
    
  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // m_autoCommand = new TrajectoryCommand(m_drive, "New Path", 8, 5);
    // return m_autoCommand;
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
