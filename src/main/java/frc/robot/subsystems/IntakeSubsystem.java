package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.basesubsystem.MotorControllerSubsystem;
import frc.lib.basesubsystem.PneumaticSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    public static class Constants {
        public MotorControllerSubsystem.Constants motor;
        public PneumaticSubsystem.Constants pneumatics;
    }
    private MotorControllerSubsystem m_motor;
    private PneumaticSubsystem m_pneumatics;

    private boolean m_running = false;

    public IntakeSubsystem(Constants constants) {
        this.m_motor = new MotorControllerSubsystem(constants.motor);
        this.m_pneumatics = new PneumaticSubsystem(constants.pneumatics);
        //m_pneumatics.set(Value.kOff);
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

    public void toggleRun() {
        if(m_running) off();
        else on();
    }

    public void on() {
        m_running = true;
        m_motor.set(0.8);
    }

    public void off() {
        m_running = false;
        m_motor.set(0);
    }
}
