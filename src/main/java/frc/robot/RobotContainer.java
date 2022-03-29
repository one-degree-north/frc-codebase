// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.TankDriveSubsystem;
import frc.lib.encoder.ODN_CANCoder;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  private TankDriveSubsystem m_drive = new TankDriveSubsystem(Constants.driveConstants);

  private MotorControllerSubsystem m_intake = new MotorControllerSubsystem(Constants.intakeConstants);
  private MotorControllerSubsystem m_shooter = new MotorControllerSubsystem(Constants.shooterConstants);

  private MotorControllerSubsystem m_climb = new MotorControllerSubsystem(Constants.climbConstants);
  private ODN_CANCoder m_climbEncoder = new ODN_CANCoder(10); //tentative
  // Controllers here:
  private XboxController m_controller = new XboxController(0);
  // Robot commands go here:
  // This command runs on autonomous

  private double maintain = 0;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(() -> {
        m_drive.arcadeDrive(modifyAxis(m_controller.getRightX()*(Math.abs(m_controller.getRightX()))*0.8), 
          modifyAxis(m_controller.getLeftY()*0.8));
      }, m_drive
      ));

    
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
    // JoystickButton lockButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    // lockButton.whenPressed(new InstantCommand(()->m_drive.setLock(true)));

    // JoystickButton unlockButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    // lockButton.whenPressed(new InstantCommand(()->m_drive.setLock(false)));

    JoystickButton intakeIn = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    intakeIn.whenPressed(new InstantCommand(()->{
      m_intake.set(0.5);
    }, m_intake));
    intakeIn.whenReleased(new InstantCommand(()->{
      m_intake.set(0);
    }, m_intake));

    JoystickButton intakeOut = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    intakeOut.whenPressed(new InstantCommand(()->{
      m_intake.set(-0.5);
    }, m_intake));
    intakeOut.whenReleased(new InstantCommand(()->{
      m_intake.set(0);
    }, m_intake));

    JoystickButton shoot = new JoystickButton(m_controller, XboxController.Button.kX.value);
    shoot.whenPressed(new InstantCommand(()->{
      m_shooter.set(0.5);
    }, m_shooter));
    shoot.whenReleased(new InstantCommand(()->{
      m_shooter.set(0);
    }, m_shooter));
  
  //WHAT DOES MAINTAIN DO
    Trigger linearUp = new Trigger(()->m_controller.getPOV()==0);
    linearUp.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_climbEncoder.getPosition()>Constants.AutoConstants.kClimbLinearMaxPosition){
        m_climb.set(0.75);
      }
      else{
        m_climb.set(0.05);
    
      }
      maintain = 0.05;
      }, m_climb));
  
    linearUp.whenInactive(new InstantCommand(()->m_climb.set(maintain), m_climb));

    Trigger linearDown = new Trigger(()->m_controller.getPOV()==180);
    linearDown.whileActiveContinuous(new InstantCommand(()->{ 
      if(m_climbEncoder.getPosition()<Constants.AutoConstants.kClimbLinearMinPosition){
        m_climb.set(-0.5);
        }
      else{
        m_climb.set(0.05);
        
      }
      maintain = 0.05;
       
  
    }, m_climb));

    linearDown.whenInactive(new InstantCommand(()->m_climb.set(maintain), m_climb));

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