package frc.lib;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ODN_Drivebase extends Subsystem {
    /**
     * Makes the drivebase rotate in speed
     * @param speed The robot's turning rate around the z axis [-1, 1]. Clockwise is positive.
     */
    public void rotate(double speed);
    
    /**
     * Makes the drivebase stop moving
     */
    public void stop();

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
    public void resetOdometry(Pose2d initialPose);

    /**
     * @return direction the drivebase is facing in degrees [-180, 180]. Clockwise is positive.
     */
    public double getYaw();
    
    /**
     * Drives the robot forward and rotates
     * @param forward forward speed [-1, 1]
     * @param rotate turning rate around the z axis [-1, 1]
     */
    public void driveForward(double forward, double rotate);

    /**
     * Creates a trajectory for this drivebase using
     * a starting pose, an ending pose, and waypoints in between.
     * @param startPose The starting pose of the robot (sets relative coordinate system)
     * @param waypoints The points for the robot to reach before the end position
     * @param endPose The ending pose
     * @return A trajectory representing the input data
     */
    public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose);
    
    /**
     * Creates a command to make the drivebase follow a given trajectory
     * DO NOT USE THIS COMMAND DIRECTLY. It is only to be used internally
     * by the #DriveTrajectoryCommand.
     * @param trajectory
     * @return
     */
	public Command generateTrajectoryCommand(Trajectory trajectory);

}
