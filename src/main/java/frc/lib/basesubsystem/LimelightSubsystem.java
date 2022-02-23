// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  /**
   * Creates a linear attenuation function.
   * @param max_angle The maximum angle returned by the limelight
   * @return The attenuation function
   */
  public static Function<Double, Double> linearAttenuation(double max_angle) {
    return (x)->{return -x/max_angle;};
  }

  /**
   * Creates a quadratic attenuation function. a+b must add up to 1.
   * @param a The quadratic coefficient. Must be positive.
   * @param b The linear coefficient. Must be positive.
   * @param max_angle The maximum angle returned by the limelight
   * @return The attenuation function
   */
  public static Function<Double, Double> quadraticAttenuation(double a, double b, double max_angle) {
    assert a+b==1;
    return (x)->{return -x/max_angle * b - Math.signum(x) * x/max_angle * x/max_angle * a;};
  }

  private NetworkTable m_table;




  public LimelightSubsystem() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getOffsetHorizontal() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public double getOffsetVertical() {
    return m_table.getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return m_table.getEntry("ta").getDouble(0);
  }

  public double getSkew() {
    return m_table.getEntry("ts").getDouble(0);
  }

  public boolean foundTarget() {
    return m_table.getEntry("tv").getDouble(0)!=0;
  }

  public void setLED(boolean on) {
    m_table.getEntry("ledMode").setDouble(on?3:1);
     
  }

  public void setPipeline(int pipeNum) {
    m_table.getEntry("pipeline").setDouble(pipeNum);
  }
}
