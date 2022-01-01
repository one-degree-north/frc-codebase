package frc.lib;

public interface ODN_HolonomicDrivebase extends ODN_Drivebase {
    
    public void cartesianDriveAbsolute(double xSpeed, double ySpeed, double rotate);

    public void cartesianDriveRelative(double xSpeed, double ySpeed, double rotate);

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
