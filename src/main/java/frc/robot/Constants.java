// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.basesubsystem.ArmSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;
import frc.lib.basesubsystem.SwerveDriveSubsystem;
import frc.lib.basesubsystem.TankDriveSubsystem;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.encoder.ODN_NullEncoder;
import frc.lib.encoder.ODN_PhoenixEncoder;
import frc.lib.gyro.ODN_AHRS;
import frc.lib.gyro.ODN_NullGyro;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;
import frc.lib.motorcontroller.ODN_SparkMax;
import frc.lib.motorcontroller.ODN_TalonFX;
import frc.lib.motorcontroller.ODN_SparkMax.MotorType;
import frc.robot.subsystems.DoubleArmClawSubsystem;

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

 //TODO: CHECK CONSTANTS
public final class Constants {
    public static DoubleArmClawSubsystem.Constants doubleArmClawConstants = new DoubleArmClawSubsystem.Constants();

    static {
        // Tune PID and FeedForward and check CANIDs
        doubleArmClawConstants.joint1 = new ArmSubsystem.Constants();
        doubleArmClawConstants.joint1.motor = new ODN_TalonFX(4);
        doubleArmClawConstants.joint1.encoder = new ODN_CANCoder(5);
        doubleArmClawConstants.joint1.encoderFactor = 1;

        doubleArmClawConstants.joint1.maxVelocity = 1;
        doubleArmClawConstants.joint1.maxAcceleration = 1;

        doubleArmClawConstants.joint1.kp = 1;
        doubleArmClawConstants.joint1.ki = 0.0001;
        doubleArmClawConstants.joint1.kd = 10;

        doubleArmClawConstants.joint1.ks = 0;
        doubleArmClawConstants.joint1.kcos = 0;
        doubleArmClawConstants.joint1.kv = 0;

        // Tune PID and FeedForward and check CANIDs
        doubleArmClawConstants.joint2 = new ArmSubsystem.Constants();
        doubleArmClawConstants.joint2.motor = new ODN_TalonFX(6);
        doubleArmClawConstants.joint2.encoder = new ODN_CANCoder(7);
        doubleArmClawConstants.joint2.encoderFactor = 1;
        
        doubleArmClawConstants.joint2.maxVelocity = 1;
        doubleArmClawConstants.joint2.maxAcceleration = 1;

        doubleArmClawConstants.joint2.kp = 1;
        doubleArmClawConstants.joint2.ki = 0.0001;
        doubleArmClawConstants.joint2.kd = 10;

        doubleArmClawConstants.joint2.ks = 0;
        doubleArmClawConstants.joint2.kcos = 0;
        doubleArmClawConstants.joint2.kv = 0;

        doubleArmClawConstants.claw = new ODN_SparkMax(8, MotorType.brushless);
        doubleArmClawConstants.clawEncoder = new ODN_CANCoder(9);
        
        doubleArmClawConstants.joint1AbsEncoderInitPos = 0;
        doubleArmClawConstants.joint1AbsEncoderFinPos = 10;
    
        doubleArmClawConstants.joint2AbsEncoderInitPos = 0;
        doubleArmClawConstants.joint2AbsEncoderFinPos = 10;
        
        doubleArmClawConstants.clawAbsEncoderInitPos = 0;
        doubleArmClawConstants.clawAbsEncoderFinPos = 10;
    }

    public static TankDriveSubsystem.Constants tankDriveConstants = new TankDriveSubsystem.Constants();
    static {
        tankDriveConstants.left = new ODN_MotorControllerGroup(new ODN_SparkMax(0, MotorType.brushed), new ODN_SparkMax(1, MotorType.brushed));
        tankDriveConstants.right = new ODN_MotorControllerGroup(new ODN_SparkMax(2, MotorType.brushed), new ODN_SparkMax(3, MotorType.brushed));
        
        // Null Gyros and Encoders
        tankDriveConstants.leftEncoder = new ODN_NullEncoder();
        tankDriveConstants.rightEncoder = new ODN_NullEncoder();
        tankDriveConstants.gyro = new ODN_NullGyro();
        tankDriveConstants.leftEncoderFactor = 0;
        tankDriveConstants.rightEncoderFactor = 0;

        tankDriveConstants.ksVolts = 1;
        tankDriveConstants.kvVoltSecondsPerMeter = 0;
        tankDriveConstants.kaVoltSecondsSquaredPerMeter = 0;

        // This value was taken off the 2022 Season Cupid bot
        tankDriveConstants.kDriveKinematics = new DifferentialDriveKinematics(0.55);
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
