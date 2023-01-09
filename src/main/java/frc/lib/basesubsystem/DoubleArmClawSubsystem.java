// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.basesubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.encoder.ODN_CANCoder;
import frc.lib.motorcontroller.ODN_MotorController;

// Notes: ArmSubsystem must use CANCoders since absolute position is used.
// This code has never been tested and is unpolished. It was originally made
// for the 2022 Halloween community event, but the arm was never made.

public class DoubleArmClawSubsystem extends SubsystemBase {
  /** Creates a new DoubleArmClawSubsystem. */
  public static class Constants {
    public ArmSubsystem.Constants joint1;
    public ArmSubsystem.Constants joint2;
    public ODN_MotorController claw;
    public ODN_CANCoder clawEncoder;

    public double joint1AbsEncoderInitPos;
    public double joint1AbsEncoderFinPos;

    public double joint2AbsEncoderInitPos;
    public double joint2AbsEncoderFinPos;
    
    public double clawAbsEncoderInitPos;
    public double clawAbsEncoderFinPos;
  }

  private ArmSubsystem m_joint1;
  private ArmSubsystem m_joint2;
  private ODN_MotorController m_claw;
  private ODN_CANCoder m_clawEncoder;

  private double m_joint1AbsEncoderInitPos;
  private double m_joint1AbsEncoderFinPos;

  private double m_joint2AbsEncoderInitPos;
  private double m_joint2AbsEncoderFinPos;

  public double m_clawAbsEncoderInitPos;
  public double m_clawAbsEncoderFinPos;

  private boolean clawOpen;
  private final double CLAWSPEED = 0.1;

  public DoubleArmClawSubsystem(Constants constants) {
    m_joint1 = new ArmSubsystem(constants.joint1);
    m_joint2 = new ArmSubsystem(constants.joint2);
    m_claw = constants.claw;
    m_clawEncoder = constants.clawEncoder;

    m_joint1AbsEncoderInitPos = constants.joint1AbsEncoderInitPos;
    m_joint1AbsEncoderFinPos = constants.joint1AbsEncoderFinPos;

    m_joint2AbsEncoderInitPos = constants.joint2AbsEncoderInitPos;
    m_joint2AbsEncoderFinPos = constants.joint2AbsEncoderFinPos;

    m_clawAbsEncoderInitPos = constants.clawAbsEncoderInitPos;
    m_clawAbsEncoderFinPos = constants.clawAbsEncoderFinPos;
  }

  public void setGoalLocation(boolean direction) {
    // If false, sets both arms to initial position; if true, sets both arms to final position
    if (!direction) {
      m_joint1.setGoalLocation(m_joint1AbsEncoderInitPos);
      m_joint2.setGoalLocation(m_joint2AbsEncoderInitPos);
    } else {
      m_joint1.setGoalLocation(m_joint1AbsEncoderFinPos);
      m_joint2.setGoalLocation(m_joint2AbsEncoderFinPos);
    }
  }

  public void setClaw(boolean open) {
    // If true, opens claw; If false, closes claw
    clawOpen = open;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (clawOpen && m_clawEncoder.getAbsolutePosition() < m_clawAbsEncoderFinPos) {
      m_claw.set(CLAWSPEED);
    } else if (!clawOpen && m_clawEncoder.getAbsolutePosition() > m_clawAbsEncoderInitPos) {
      m_claw.set(-CLAWSPEED);
    } else {
      m_claw.set(0);
    }
  }
}
