// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.*;

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

    public static TankDriveSubsystem.Constants drive_constants = new TankDriveSubsystem.Constants();
    static {
        drive_constants.id_fl = 1;
        drive_constants.id_bl = 2;
        drive_constants.id_fr = 3;
        drive_constants.id_br = 4;
        drive_constants.id_gyro = 5;

        // These numbers must be correctly calculated
        drive_constants.ksVolts = 1;
        drive_constants.kvVoltSecondsPerMeter = 0;
        drive_constants.kaVoltSecondsSquaredPerMeter = 0;

        drive_constants.kDriveKinematics = new DifferentialDriveKinematics(0.1);
    }
    
    public static int kControllerPort = 0;
    
    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 2;
        public static double kMaxAccelerationMetersPerSecondSquared = 2;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = 16*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 256*Math.PI*Math.PI;
        
        // These numbers must be correctly calculated
        public static double kRamseteB = 1;
        public static double kRamseteZeta = 1;
		public static double kPYController = 1;
		public static double kPXController = 1;
		public static Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
		public static double kPThetaController = 1;
    }

}
