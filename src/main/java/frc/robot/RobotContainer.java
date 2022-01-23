// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.basesubsystem.LimelightSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.robot.commands.ElevatorHeightCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer container;
  // Robot subsystems here:
  // private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private LimelightSubsystem m_limelight = new LimelightSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.shooterConstants);
  private IndexerSubsystem m_indexer = new IndexerSubsystem(Constants.indexerConstants);
  private IntakeSubsystem m_intake = new IntakeSubsystem(Constants.intakeConstants);
  private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:


  // This command runs on autonomous
  private Command m_autoCommand = null;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    container = this;

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
      m_indexer.on();
    },
    m_indexer));
    
    m_climb.disable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton shoot = new JoystickButton(m_controller, XboxController.Button.kA.value);
    shoot.whenPressed(new InstantCommand(()-> m_shooter.on(), m_shooter));
    shoot.whenReleased(new InstantCommand(()-> m_shooter.off(), m_shooter));

    JoystickButton climb = new JoystickButton(m_controller, XboxController.Button.kY.value);
    climb.whenPressed(new SequentialCommandGroup(
      new InstantCommand(()->m_climb.enable(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP),
      //Drive robot back a little
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.BOTTOM),
      new InstantCommand(()->m_climb.contractRotation(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TRANSFER),
      new InstantCommand(()->m_climb.extendRotation(), m_climb),
      new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP)
    ));

    //TODO: Remove when automation confirmed works
    JoystickButton moveIntake = new JoystickButton(m_controller, XboxController.Button.kB.value);
    moveIntake.whenPressed(new InstantCommand(()-> m_intake.toggle(), m_intake));
    JoystickButton spinIntake = new JoystickButton(m_controller, XboxController.Button.kX.value);
    spinIntake.whenPressed(new InstantCommand(()-> m_intake.on(), m_intake));
    spinIntake.whenReleased(new InstantCommand(()-> m_intake.off(), m_intake));
    
  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
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

  public IndexerSubsystem getIndexer() {
    return m_indexer;
  }

  public IntakeSubsystem getIntake() {
    return m_intake;
  }

  public ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public SwerveDriveSubsystem getDrivebase() {
    return null;
  }

  public LimelightSubsystem getLimelight() {
    return m_limelight;
  }

  public XboxController getJoystick() {
    return m_controller;
  }
}
