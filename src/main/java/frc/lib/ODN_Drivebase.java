package frc.lib;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
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

    public double getYaw();
    
    public void driveForward(double forward, double rotate);

    /**
     * Creates a trajectory command for this drivebase using
     * a starting pose, an ending pose, and waypoints in between.
     * DO NOT USE THIS COMMAND DIRECTLY. It is only to be used internally
     * by the DriveTrajectoryCommand.
     * @param startPose The starting pose of the robot (sets relative coordinate system)
     * @param waypoints The points for the robot to reach before the end position
     * @param endPose The ending pose (relative to the starting pose)
     * @return A trajectory command to be used by DriveTrajectoryCommand
     */
    public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose);
    
	public Command generateTrajectoryCommand(Trajectory trajectory);

}
