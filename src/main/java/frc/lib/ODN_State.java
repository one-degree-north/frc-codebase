package frc.lib;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ODN_State extends Subsystem {
  /**
   * Sets a new goal position for the subsystem
   * @param pos New goal position
   */
  public void setGoalLocation(double pos);

  /**
   * Checks whether the subsystem has reached the goal location
   * @return true if subsystem has reached goal location
   */
  public boolean atGoalLocation();

  public void resetPosition();

}
