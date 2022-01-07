package frc.lib;

public interface ODN_HolonomicDrivebase extends ODN_Drivebase {
    
    /**
     * Drive the robot with cartesian coordinate inputs field-oriented
     * @param xSpeed speed along the x-axis [-1, 1]
     * @param ySpeed speed along the y-axis [-1, 1]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate);

    /**
     * Drive the robot with cartesian coordinate inputs robot-oriented
     * @param xSpeed speed along the x-axis [-1, 1]
     * @param ySpeed speed along the y-axis [-1, 1]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate);

    /**
     * Drive the robot with polar coordinate inputs
     * @param magnitude speed to move at [-1, 1]
     * @param angle direction to move in degrees [-180, 180]
     * @param rotate rotation speed around the z-axis [-1, 1]. Clockwise is positive.
     */
    public void polarDrive(double magnitude, double angle, double rotate);

    public default void rotate(double speed) {
        cartesianDriveAbsolute(0, 0, speed);
    }
    
    public default void stop() {
        cartesianDriveAbsolute(0, 0, 0);
    }
    
    public default void driveForward(double forward, double rotate) {
		polarDrive(forward, getYaw(), rotate);
    }
}
