// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_TalonSRX;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.lib.basesubsystem.TankDriveSubsystem;
import frc.lib.encoder.ODN_NullEncoder;
import frc.lib.gyro.ODN_NullGyro;

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

    public static TankDriveSubsystem.Constants driveConstants = new TankDriveSubsystem.Constants();
    static {
        driveConstants.left = new ODN_MotorControllerGroup(new ODN_TalonSRX(12), new ODN_TalonSRX(8));
        driveConstants.right = new ODN_MotorControllerGroup(new ODN_TalonSRX(14), new ODN_TalonSRX(6));
        driveConstants.leftEncoder = new ODN_NullEncoder();
        driveConstants.rightEncoder = new ODN_NullEncoder();
        driveConstants.gyro = new ODN_NullGyro();
        driveConstants.leftEncoderFactor = 0;
        driveConstants.rightEncoderFactor = 0;

        driveConstants.ksVolts = 1;
        driveConstants.kvVoltSecondsPerMeter = 0;
        driveConstants.kaVoltSecondsSquaredPerMeter = 0;
        driveConstants.kDriveKinematics = new DifferentialDriveKinematics(0.55);
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
