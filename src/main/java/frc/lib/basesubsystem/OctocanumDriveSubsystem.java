package frc.lib.basesubsystem;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.ODN_HolonomicDrivebase;
import frc.lib.encoder.ODN_Encoder;
import frc.lib.gyro.ODN_Gyro;
import frc.lib.motorcontroller.ODN_MotorController;
import frc.lib.motorcontroller.ODN_MotorControllerGroup;

public class OctocanumDriveSubsystem extends ODN_HolonomicDrivebase {
    public static class Constants {
        // MotorControllers
        public ODN_MotorController leftFront;
        public ODN_MotorController leftBack;
        public ODN_MotorController rightFront;
        public ODN_MotorController rightBack;
        // Encoders
        public ODN_Encoder leftFrontEncoder;
        public ODN_Encoder leftBackEncoder;

        public ODN_Encoder rightFrontEncoder;
        public ODN_Encoder rightBackEncoder;

        // gyro
        public ODN_Gyro gyro;

        /** Converts output from encoder to meters travelled on the left side */
        public double leftFrontEncoderFactor;
        public double leftBackEncoderFactor;

        /** Converts output from encoder to meters travelled on the right side */
        public double rightFrontEncoderFactor;
        public double rightBackEncoderFactor;

        // These three are factors for the feedforward calculations
        public double ksVolts;
        public double kvVoltSecondsPerMeter;
        public double kaVoltSecondsSquaredPerMeter;

        /**
         * PID proportional factor for Trajectories. Should be set to one since velocity
         * is in meters per second
         */
        public double kPFrontLeftVel = 1;
        public double kPBackLeftVel = 1;
        public double kPFrontRightVel = 1;
        public double kPBackRightVel = 1;

        public int kPDriveVel = 1;

        public double trackWidth;
        public double wheelBase;

        private PneumaticSubsystem pneumaticSwitch;
    }

    private TankDriveSubsystem tankDrive;
    private MecanumDriveSubsystem mecanumDrive;
    private PneumaticSubsystem pneumaticSwitch;

    /** Creates a new OctocanumDriveSubsystem. */
    public OctocanumDriveSubsystem(Constants constants) {
        super(constants.gyro);
        TankDriveSubsystem.Constants tankContants = new TankDriveSubsystem.Constants();
        tankContants.left = new ODN_MotorControllerGroup(constants.leftFront, constants.leftBack);
        tankContants.right = new ODN_MotorControllerGroup(constants.rightFront, constants.rightBack);
        tankContants.leftEncoder = constants.leftFrontEncoder;
        tankContants.rightEncoder = constants.rightFrontEncoder;
        tankContants.kPDriveVel = constants.kPDriveVel;
        tankContants.ksVolts = constants.ksVolts;
        tankContants.kvVoltSecondsPerMeter = constants.kvVoltSecondsPerMeter;
        tankContants.kaVoltSecondsSquaredPerMeter = constants.kaVoltSecondsSquaredPerMeter;
        tankContants.gyro = constants.gyro;
        tankContants.leftEncoderFactor = constants.leftFrontEncoderFactor;
        tankContants.rightEncoderFactor = constants.rightFrontEncoderFactor;
        tankContants.kDriveKinematics = new DifferentialDriveKinematics(constants.trackWidth);

        tankDrive = new TankDriveSubsystem(tankContants);

        MecanumDriveSubsystem.Constants mecanumConstants = new MecanumDriveSubsystem.Constants();
        mecanumConstants.frontLeft = constants.leftFront;
        mecanumConstants.frontRight = constants.rightFront;
        mecanumConstants.rearLeft = constants.leftBack;
        mecanumConstants.rearRight = constants.rightBack;
        mecanumConstants.frontLeftEncoder = constants.leftFrontEncoder;
        mecanumConstants.frontRightEncoder = constants.rightFrontEncoder;
        mecanumConstants.rearLeftEncoder = constants.leftBackEncoder;
        mecanumConstants.rearRightEncoder = constants.rightBackEncoder;
        mecanumConstants.ksVolts = constants.ksVolts;
        mecanumConstants.kvVoltSecondsPerMeter = constants.kvVoltSecondsPerMeter;
        mecanumConstants.kaVoltSecondsSquaredPerMeter = constants.kaVoltSecondsSquaredPerMeter;
        mecanumConstants.gyro = constants.gyro;
        mecanumConstants.frontLeftEncoderFactor = constants.leftFrontEncoderFactor;
        mecanumConstants.frontRightEncoderFactor = constants.rightFrontEncoderFactor;
        mecanumConstants.rearLeftEncoderFactor = constants.leftBackEncoderFactor;
        mecanumConstants.rearRightEncoderFactor = constants.rightBackEncoderFactor;
        mecanumConstants.kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(-constants.trackWidth / 2, constants.wheelBase / 2),
                new Translation2d(constants.trackWidth / 2, constants.wheelBase / 2),
                new Translation2d(-constants.trackWidth / 2, -constants.wheelBase / 2),
                new Translation2d(constants.trackWidth / 2, -constants.wheelBase / 2));

        mecanumDrive = new MecanumDriveSubsystem(mecanumConstants);
        pneumaticSwitch = constants.pneumaticSwitch;
    }

    public void arcadeDrive(double forward, double rotation) {
        pneumaticSwitch.set(Value.kOff);
        tankDrive.arcadeDrive(forward, rotation);
    }

    public void tankDrive(double left, double right) {
        pneumaticSwitch.set(Value.kOff);
        tankDrive.tankDrive(left, right);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate) {
        pneumaticSwitch.set(Value.kForward);
        mecanumDrive.cartesianDriveAbsolute(ySpeed, xSpeed, rotate);
    }

    @Override
    public void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate) {
        pneumaticSwitch.set(Value.kForward);
        mecanumDrive.cartesianDriveRelative(ySpeed, xSpeed, rotate);
    }

    @Override
    public void polarDrive(double magnitude, Rotation2d direction, double rotate) {
        pneumaticSwitch.set(Value.kForward);
        mecanumDrive.polarDrive(magnitude, direction, rotate);
    }

    @Override
    public void resetOdometry(Pose2d initialPose) {
        mecanumDrive.resetOdometry(initialPose);
    }

    @Override
    public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
        return mecanumDrive.generateTrajectory(startPose, waypoints, endPose);
    }

    @Override
    public Command generateTrajectoryCommand(Trajectory trajectory) {
        return mecanumDrive.generateTrajectoryCommand(trajectory);
    }
}
