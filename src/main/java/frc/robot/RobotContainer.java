// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LimelightArcCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MotorControllerSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.TrajectoryCommand;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.sensor.ODN_ColorSensor;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private LimelightSubsystem m_limelight = new LimelightSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.shooterConstants);
  private MotorControllerSubsystem m_indexerMotor = new MotorControllerSubsystem(Constants.indexerConstants);
  private IndexerSubsystem m_indexer = new IndexerSubsystem(m_indexerMotor, new ODN_ColorSensor());
  private MotorControllerSubsystem m_intakeMotor = new MotorControllerSubsystem(Constants.intakeConstants);
  private PneumaticSubsystem m_intakePneumatic = new PneumaticSubsystem(Constants.pneumaticConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous
  private Command m_autoCommand = null;
  private ODN_TalonFX testFX = new ODN_TalonFX(15);


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

    m_indexer.setDefaultCommand(new RunCommand(()-> {
      m_indexer.set(0.8);
    },
    m_indexer));

    ShuffleboardTab tab = Shuffleboard.getTab("chungus");
    tab.addNumber("chungus", ()->0);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton shooterSpeed = new JoystickButton(m_controller, XboxController.Button.kA.value);
    shooterSpeed.whenPressed(new InstantCommand(()-> m_shooter.on(), m_shooter));
    shooterSpeed.whenReleased(new InstantCommand(()-> m_shooter.off(), m_shooter));

    JoystickButton moveIntake = new JoystickButton(m_controller, XboxController.Button.kB.value);
    moveIntake.whenPressed(new InstantCommand(()-> m_intakePneumatic.toggle(), m_intakePneumatic));

    JoystickButton spinIntake = new JoystickButton(m_controller, XboxController.Button.kX.value);
    spinIntake.whenHeld(new InstantCommand(()-> m_intakeMotor.set(0.8), m_intakeMotor));


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
