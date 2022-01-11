// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;

import frc.lib.gyro.ODN_AHRS;
import frc.lib.helper.SwerveModuleBuilder;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.MotorControllerSubsystem;

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

    public static SwerveDriveSubsystem.Constants swerveConstants = new SwerveDriveSubsystem.Constants();
    static {
        swerveConstants.DRIVETRAIN_TRACKWIDTH_METERS = 0.47;
        swerveConstants.DRIVETRAIN_WHEELBASE_METERS = 0.47;

        swerveConstants.frontLeftModule = SwerveModuleBuilder.createNEO_MK3_STANDARD(
                "Front Left Module", 0, 0,
                2, 1, 12, -Math.toRadians(7.471));

        swerveConstants.frontRightModule = SwerveModuleBuilder.createNEO_MK3_STANDARD(
                "Front Right Module", 2, 0,
                6, 5, 11, -Math.toRadians(136.055 - 180));

        swerveConstants.backLeftModule = SwerveModuleBuilder.createNEO_MK3_STANDARD(
                "Back Left Module", 4, 0,
                4, 3, 9, -Math.toRadians(71.542 - 180));

        swerveConstants.backRightModule = SwerveModuleBuilder.createNEO_MK3_STANDARD(
                "Back Right Module", 6, 0,
                8, 7, 10, -Math.toRadians(-21.533));

        swerveConstants.config = SwerveModuleBuilder.NEO_MK3_STANDARD_CONFIG;

        swerveConstants.gyro = new ODN_AHRS();
    }

    public static MotorControllerSubsystem.Constants motorConstants = new MotorControllerSubsystem.Constants();
    static {
        motorConstants.motor = new ODN_TalonFX(15);
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
