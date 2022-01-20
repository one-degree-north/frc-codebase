// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.sensor.ODN_ColorSensor;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DIST_TO_SHOOT_FROM = 0;

    // public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    // static {
    //     swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 0.47;
    //     swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 0.47;
    
    //     swerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    //     swerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(7.471);
    
    //     swerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    //     swerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(136.055-180);
    
    //     swerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    //     swerveConstants.BACK_LEFT_MODULE_STEER_MOTOR = 3;
    //     swerveConstants.BACK_LEFT_MODULE_STEER_ENCODER = 9;
    //     swerveConstants.BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71.542-180);

    //     swerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    //     swerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-21.533);

    //     swerveConstants.gyro = new ODN_AHRS();
    // }

    // -----------------------------------------------------------------------
    // --------- SHOOTER -----------------------------------------------------
    // -----------------------------------------------------------------------

    public static MotorControllerSubsystem.Constants shooterMotorConstants = new MotorControllerSubsystem.Constants();
    static {
        ODN_TalonFX motor = new ODN_TalonFX(14);
        motor.setInverted(true);
        shooterMotorConstants.motor = new ODN_MotorControllerGroup(
            new ODN_TalonFX(15),
            motor
        );
    }

    public static ShooterSubsystem.Constants shooterConstants = new ShooterSubsystem.Constants();
    static {
        shooterConstants.motor = new MotorControllerSubsystem(shooterMotorConstants);
        shooterConstants.shoot_speed = 3000;
    }

    public static MotorControllerSubsystem.Constants indexerMotorConstants = new MotorControllerSubsystem.Constants();
    static {
        indexerMotorConstants.motor = new ODN_TalonFX(16);
    }
    public static IndexerSubsystem.Constants indexerConstants = new IndexerSubsystem.Constants();
    static {
        indexerConstants.motor = new MotorControllerSubsystem(indexerMotorConstants);
        indexerConstants.color = new ODN_ColorSensor();
        indexerConstants.breakbeam_enter = new AnalogInput(1);
        indexerConstants.breakbeam_exit = new DigitalInput(2);
    }

    

    // -----------------------------------------------------------------------
    // --------- INTAKE ------------------------------------------------------
    // -----------------------------------------------------------------------

    public static MotorControllerSubsystem.Constants intakeMotorConstants = new MotorControllerSubsystem.Constants();
    static {
        intakeMotorConstants.motor = new ODN_TalonFX(17);
    }

    public static PneumaticSubsystem.Constants intakePneumaticConstants = new PneumaticSubsystem.Constants();
    static {
        intakePneumaticConstants.solenoids = new DoubleSolenoid[]{
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1),
            new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3)
        };
    }

    public static IntakeSubsystem.Constants intakeConstants = new IntakeSubsystem.Constants();
    static {
        intakeConstants.motor = new MotorControllerSubsystem(intakeMotorConstants);
        intakeConstants.pneumatics = new PneumaticSubsystem(intakePneumaticConstants);
    }

    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 1;
        public static double kMaxAccelerationMetersPerSecondSquared = 1;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 40;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 40;
        
        // These numbers must be correctly calculated
        public static double kRamseteB = 1;
        public static double kRamseteZeta = 1;
		public static double kPYController = 1;
		public static double kPXController = 1;
		public static double kPThetaController = 1;
    }

}
