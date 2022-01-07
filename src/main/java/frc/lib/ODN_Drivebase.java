package frc.lib;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.gyro.ODN_Gyro;

public abstract class ODN_Drivebase extends SubsystemBase {
    private ODN_Gyro m_gyro;

    public ODN_Drivebase(ODN_Gyro gyro) {
        this.m_gyro = gyro;
    }

    /**
     * Makes the drivebase rotate in speed
     * @param speed The robot's turning rate around the z axis [-1, 1]. Clockwise is positive.
     */
    public abstract void rotate(double speed);
    
    /**
     * Makes the drivebase stop moving
     */
    public abstract void stop();

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
    public abstract void resetOdometry(Pose2d initialPose);

    /**
     * @return direction the drivebase is facing
     */
    public Rotation2d getYaw() {
        return m_gyro.getYaw();
    }
    
    /**
     * Resets the current heading to 0 degrees
     */
    public void resetYaw() {
        m_gyro.resetYaw();
    }
    
    /**
     * Drives the robot forward and rotates
     * @param forward forward speed [-1, 1]
     * @param rotate turning rate around the z axis [-1, 1]
     */
    public abstract void driveForward(double forward, double rotate);

    /**
     * Creates a trajectory for this drivebase using
     * a starting pose, an ending pose, and waypoints in between.
     * DO NOT USE THIS COMMAND DIRECTLY. It is only to be used internally
     * by the #DriveTrajectoryCommand.
     * @param startPose The starting pose of the robot (sets relative coordinate system)
     * @param waypoints The points for the robot to reach before the end position
     * @param endPose The ending pose
     * @return A trajectory representing the input data
     */
    public abstract Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose);
    
    /**
     * Creates a command to make the drivebase follow a given trajectory
     * DO NOT USE THIS COMMAND DIRECTLY. It is only to be used internally
     * by the #DriveTrajectoryCommand.
     * @param trajectory
     * @return
     */
	public abstract Command generateTrajectoryCommand(Trajectory trajectory);

}
