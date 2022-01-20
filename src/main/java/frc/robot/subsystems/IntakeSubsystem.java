package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    public static class Constants {
        public MotorControllerSubsystem motor;
        public PneumaticSubsystem pneumatics;
    }
    private MotorControllerSubsystem m_motor;
    private PneumaticSubsystem m_pneumatics;

    public IntakeSubsystem(Constants constants) {
        this.m_motor = constants.motor;
        this.m_pneumatics = constants.pneumatics;
    }

    public void toggle() {
        m_pneumatics.toggle();
    }

    public void raise() {
        m_pneumatics.set(Value.kReverse);
    }

    public void drop() {
        m_pneumatics.set(Value.kForward);
    }

    public void on() {
        m_motor.set(0.8);
    }

    public void off() {
        m_motor.set(0);
    }
}
