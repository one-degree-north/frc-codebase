package frc.lib.motorcontroller;

import com.revrobotics.CANSparkMax;

import frc.lib.encoder.Encoder;
import frc.lib.encoder.ODN_CANEncoder;

public class ODN_SparkMax implements MotorController {

    public static enum MotorType {
        brushed, brushless
    }

    private CANSparkMax m_backend;

    public ODN_SparkMax(int CAN_ID, MotorType type) {
        m_backend = new CANSparkMax(CAN_ID,
                type == MotorType.brushed ? com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
                        : com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    @Override
    public void set(double speed) {
        m_backend.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        m_backend.setVoltage(voltage);
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_backend.setInverted(isInverted);
    }

    @Override
    public Encoder getEncoder() {
        return new ODN_CANEncoder(m_backend.getEncoder());
    }

    @Override
    public CANSparkMax getBackend() {
        return m_backend;
    }
    
}
