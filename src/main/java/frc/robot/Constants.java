// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.basesubsystem.TankDriveSubsystem;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.encoder.ODN_NullEncoder;
import frc.lib.gyro.ODN_AHRS;
import frc.lib.gyro.ODN_NullGyro;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;
import frc.robot.subsystems.ClimbSubsystem;
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
    public static TankDriveSubsystem.Constants tankConstants = new TankDriveSubsystem.Constants();
    static {
        tankConstants.left = new ODN_MotorControllerGroup(new ODN_SparkMax(1, MotorType.brushed), new ODN_SparkMax(3, MotorType.brushed));
        tankConstants.right = new ODN_MotorControllerGroup(new ODN_SparkMax(2, MotorType.brushed), new ODN_SparkMax(4, MotorType.brushed));
        tankConstants.leftEncoder = new ODN_NullEncoder();
        tankConstants.rightEncoder = new ODN_NullEncoder();
        tankConstants.gyro = new ODN_NullGyro();
        tankConstants.leftEncoderFactor = 0;
        tankConstants.rightEncoderFactor = 0;

        tankConstants.ksVolts = 1;
        tankConstants.kvVoltSecondsPerMeter = 0;
        tankConstants.kaVoltSecondsSquaredPerMeter = 0;
        tankConstants.kDriveKinematics = new DifferentialDriveKinematics(0.55);
    }



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
