package frc.lib;

public interface ODN_HolonomicDrivebase extends ODN_Drivebase {
    
    public void cartesianDrive(double xSpeed, double ySpeed, double rotate);

    public void polarDrive(double magnitude, double angle, double rotate);

    public default void rotate(double speed) {
        cartesianDrive(0, 0, speed);
    }
    
    public default void stop() {
        cartesianDrive(0, 0, 0);
    }
    
    public default void driveForward(double forward, double rotate) {
		polarDrive(forward, getYaw(), rotate);
    }
}
