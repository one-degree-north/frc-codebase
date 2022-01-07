package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.gyro.ODN_Gyro;

public abstract class ODN_HolonomicDrivebase extends ODN_Drivebase {
    
    public ODN_HolonomicDrivebase(ODN_Gyro gyro) {
        super(gyro);
    }

    /**
     * Drive the robot with cartesian coordinate inputs field-oriented
     * @param xSpeed speed along the x-axis [-1, 1]
     * @param ySpeed speed along the y-axis [-1, 1]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public abstract void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate);

    /**
     * Drive the robot with cartesian coordinate inputs robot-oriented
     * @param xSpeed speed along the x-axis [-1, 1]
     * @param ySpeed speed along the y-axis [-1, 1]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public abstract void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate);

    /**
     * Drive the robot with polar coordinate inputs
     * @param magnitude speed to move at [-1, 1]
     * @param direction direction to move in degrees [-180, 180]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public abstract void polarDrive(double magnitude, Rotation2d direction, double rotate);

    public void rotate(double speed) {
        cartesianDriveAbsolute(0, 0, speed);
    }
    
    public void stop() {
        cartesianDriveAbsolute(0, 0, 0);
    }
    
    public void driveForward(double forward, double rotate) {
		polarDrive(forward, getYaw(), rotate);
    }
}
