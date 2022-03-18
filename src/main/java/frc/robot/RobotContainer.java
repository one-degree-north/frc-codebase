// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerContinueCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Wait;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  boolean r;
  public static RobotContainer container;
  // Robot subsystems here:
  private SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem(Constants.swerveConstants);
  private ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.shooterConstants);
  private IndexerSubsystem m_indexer = new IndexerSubsystem(Constants.indexerConstants);
  private IntakeSubsystem m_intake = new IntakeSubsystem(Constants.intakeConstants);

  private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants);

  // Controllers here:
  private XboxController m_controller = new XboxController(0);
  private Compressor compressor = new Compressor(17, PneumaticsModuleType.CTREPCM);

  // Robot commands go here:
  public Command getIntakeToggleCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          r = m_intake.isRunning();
          if (r) {
            m_intake.off();
          } else {
            m_intake.drop();
          }
        }),
        new Wait(),
        new InstantCommand(() -> {
          if (r) {
            m_intake.raise();
          } else {
            m_intake.on();
          }
        })

    );
  }

  // This command runs on autonomous
  private Command m_autoCommand = new SequentialCommandGroup(
      new InstantCommand(() -> m_climb.extendRotation(), m_climb),
      new ShootCommand(m_shooter, m_indexer),
      new InstantCommand(() -> m_climb.contractRotation(), m_climb),
      new DriveCommand(m_drive, 90 + 14.5, 105),
      new RotateCommand(m_drive, 180 - 14.5),
      getIntakeToggleCommand(),
      new DriveCommand(m_drive, 270, 22),
      getIntakeToggleCommand(),
      new RotateCommand(m_drive, -180 + 14.5),
      new DriveCommand(m_drive, -90 + 14.5, 138),
      new InstantCommand(() -> m_climb.extendRotation(), m_climb),
      new ShootCommand(m_shooter, m_indexer),
      new InstantCommand(() -> m_climb.contractRotation(), m_climb),
      new DriveCommand(m_drive, 90 + 14.5, 138)

  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    container = this;

    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> {

    m_drive.cartesianDriveRelative(modifyAxis(m_controller.getLeftY()),
    modifyAxis(m_controller.getLeftX()),
    modifyAxis(m_controller.getRightX()));
    },
    m_drive));

    m_climb.disable();
    m_climb.contractRotation();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger manualShooterRev = new JoystickButton(m_controller, XboxController.Button.kA.value);
    manualShooterRev.whenActive(() -> m_shooter.toggle(), m_shooter);
    /*
     * Trigger shootBall = new Trigger(()->m_controller.getRightTriggerAxis()>0.5);
     * Trigger intakeToggle3 = new
     * Trigger(()->m_controller.getLeftTriggerAxis()>0.5);
     * shootBall.whenActive(new IndexerContinueCommand(m_indexer));
     * Trigger intakeToggle = new JoystickButton(m_controller,
     * XboxController.Button.kRightBumper.value);
     * intakeToggle.whenActive(() -> m_indexer.onboth(), m_intake);
     * Trigger intakeToggle2 = new JoystickButton(m_controller,
     * XboxController.Button.kLeftBumper.value);
     * intakeToggle2.whenActive(() -> m_indexer.off(), m_intake);
     * intakeToggle3.whenActive(() -> m_intake.toggleRun(), m_intake);
     * `
     */

    Trigger intakeRun = new Trigger(() -> m_controller.getLeftTriggerAxis() > 0.5);
    Trigger shootBall = new Trigger(() -> m_controller.getRightTriggerAxis() > 0.5);

    Trigger t = new JoystickButton(m_controller, XboxController.Button.kB.value);
    t.whenActive(() -> {
      if (compressor.enabled()) {
        compressor.disable();
      } else {
        compressor.enableDigital();
      }
    });

    // drop or release
    intakeRun.whenActive(new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_intake.drop();
        }),
        new Wait(),
        new InstantCommand(() -> {
          m_intake.on();
        })

    ));
    intakeRun.whenInactive(new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_intake.off();
        }),
        new Wait(),
        new InstantCommand(() -> {
          m_intake.raise();
        })

    ));

    m_indexer.ballAtExit().whileActiveOnce(new RunCommand(() -> m_indexer.off(), m_indexer));

    // shoot ball
    shootBall.whenActive(new IndexerContinueCommand(m_indexer));

    // manual climber
    Trigger climberUp = new Trigger(() -> m_controller.getPOV() == 180);
    Trigger climberDown = new Trigger(() -> m_controller.getPOV() == 0);
    Trigger climberToggle = new JoystickButton(m_controller, XboxController.Button.kY.value);
    Trigger rotationToggle = new JoystickButton(m_controller, XboxController.Button.kX.value);
    double climbSpeed = 3000;

    climberToggle.whenActive(new InstantCommand(() -> m_climb.toggle()));
    rotationToggle.whenActive(new InstantCommand(() -> m_climb.extendRotation()));
    rotationToggle.whenInactive(new InstantCommand(() -> m_climb.contractRotation()));
    climberUp.whileActiveContinuous(new InstantCommand(() -> m_climb.setMotor(climbSpeed), m_climb));
    climberDown.whileActiveContinuous(new InstantCommand(() -> m_climb.setMotor(-climbSpeed), m_climb));
    climberUp.whenInactive(new InstantCommand(() -> m_climb.setMotor(0), m_climb));
    climberDown.whenInactive(new InstantCommand(() -> m_climb.setMotor(0), m_climb));

    // climbing stuff: figure this out later
    // JoystickButton climb = new JoystickButton(m_controller,
    // XboxController.Button.kY.value);
    // climb.whenPressed(new SequentialCommandGroup(
    // new InstantCommand(()->m_climb.enable(), m_climb),
    // new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP),
    // new DriveBackCommand(m_drive),
    // new ElevatorHeightCommand(m_climb, ClimbSubsystem.BOTTOM),
    // new InstantCommand(()->m_climb.contractRotation(), m_climb),
    // new ElevatorHeightCommand(m_climb, ClimbSubsystem.TRANSFER),
    // new InstantCommand(()->m_climb.extendRotation(), m_climb),
    // new ElevatorHeightCommand(m_climb, ClimbSubsystem.TOP)
    // ));

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
    return m_drive;
  }

  public XboxController getJoystick() {
    return m_controller;
  }
}