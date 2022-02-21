// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.basesubsystem.TankDriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems here:
  private TankDriveSubsystem m_drive = new TankDriveSubsystem(Constants.driveConstants);
  // Controllers here:
  private XboxController m_controller = new XboxController(0);

  // Robot commands go here:
  // This command runs on autonomous


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
<<<<<<< HEAD
    JoystickButton lockButton = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    lockButton.whenPressed(new InstantCommand(()->m_shooter.setSpeed(3000), m_shooter));
    
=======
    // JoystickButton lockButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    // lockButton.whenPressed(new InstantCommand(()->m_drive.setLock(true)));

    // JoystickButton unlockButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    // lockButton.whenPressed(new InstantCommand(()->m_drive.setLock(false)));
>>>>>>> e6c59508d758eb4bb85acf880bded1a20a1df67c
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

    /*
      ББББББББББББ         OOOOOO           ЛЛЛЛЛЛЛЛЛЛЛ    ШШШ    ШШШ    ШШШ         OOOOOO         ИИИ         ИИИИИ
      ББББББББББББ       OOOOOOOOOO         ЛЛЛЛЛЛЛЛЛЛЛ    ШШШ    ШШШ    ШШШ       OOOOOOOOOO       ИИИ        ИИИИИИ
      БББ              OOOO      OOOO       ЛЛЛ     ЛЛЛ    ШШШ    ШШШ    ШШШ     OOOO      OOOO     ИИИ       ИИИ ИИИ
      БББ             OOO          OOO      ЛЛЛ     ЛЛЛ    ШШШ    ШШШ    ШШШ    OOO          OOO    ИИИ      ИИИ  ИИИ
      БББББББББ       OOO          OOO      ЛЛЛ     ЛЛЛ    ШШШ    ШШШ    ШШШ    OOO          OOO    ИИИ     ИИИ   ИИИ
      БББББББББББ     OOO          OOO      ЛЛЛ     ЛЛЛ    ШШШ    ШШШ    ШШШ    OOO          OOO    ИИИ    ИИИ    ИИИ
      БББ      БББ    OOO          OOO     ЛЛЛ      ЛЛЛ    ШШШ    ШШШ    ШШШ    OOO          OOO    ИИИ   ИИИ     ИИИ
      БББ      БББ    OOO          OOO     ЛЛЛ      ЛЛЛ    ШШШ    ШШШ    ШШШ    OOO          OOO    ИИИ  ИИИ      ИИИ
      БББ      БББ     OOOO      OOOO     ЛЛЛ       ЛЛЛ    ШШШ    ШШШ    ШШШ     OOOO      OOOO     ИИИ ИИИ       ИИИ
      БББББББББББ        OOOOOOOOOO       ЛЛЛ       ЛЛЛ    ШШШШШШШШШШШШШШШШШ       OOOOOOOOOO       ИИИИИИ        ИИИ
      БББББББББ            OOOOOO        ЛЛЛ        ЛЛЛ    ШШШШШШШШШШШШШШШШШ         OOOOOO         ИИИИИ         ИИИ
      

      ЧЧЧ       ЧЧЧ    УУУ         УУУ   ННН        ННН    ГГГГГГГГГГГГГГ    УУУ         УУУ       СССССССССС
      ЧЧЧ       ЧЧЧ     УУУ       УУУ    ННН        ННН    ГГГГГГГГГГГГГГ     УУУ       УУУ      СССССССССССС
      ЧЧЧ       ЧЧЧ      УУУ     УУУ     ННН        ННН    ГГГ                 УУУ     УУУ     СССС
      ЧЧЧ       ЧЧЧ       УУУ   УУУ      ННН        ННН    ГГГ                  УУУ   УУУ     ССС
      ЧЧЧЧ      ЧЧЧ        УУУ УУУ       НННННННННННННН    ГГГ                   УУУ УУУ      ССС 
        ЧЧЧЧЧЧЧЧЧЧЧ         УУУУУ        НННННННННННННН    ГГГ                    УУУУУ       ССС
            ЧЧЧЧЧЧЧ          УУУ         ННН        ННН    ГГГ                     УУУ        ССС
                ЧЧЧ         УУУ          ННН        ННН    ГГГ                    УУУ         ССС
                ЧЧЧ        УУУ           ННН        ННН    ГГГ                   УУУ           СССС
                ЧЧЧ      УУУУ            ННН        ННН    ГГГ                 УУУУ              ССССССССССССС
                ЧЧЧ    УУУУ              ННН        ННН    ГГГ                УУУУ                 ССССССССССС
    */
  }
}