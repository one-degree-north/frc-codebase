package frc.lib.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import frc.lib.encoder.ODN_Encoder;
import frc.lib.encoder.ODN_CANEncoder;

public class ODN_SparkMax implements ODN_MotorController {

    public static enum Type {
        brushed, brushless
    }

    private CANSparkMax m_backend;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    public ODN_SparkMax(int CAN_ID, Type type) {
        m_backend = new CANSparkMax(CAN_ID,
                type == Type.brushed ? MotorType.kBrushed
                        : MotorType.kBrushless);
        m_encoder = m_backend.getEncoder();
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
    public ODN_Encoder getEncoder() {
        return new ODN_CANEncoder(m_encoder);
    }

    @Override
    public CANSparkMax getBackend() {
        return m_backend;
    }

    @Override
    public void setRealSpeed(double speed) {
        m_pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }
    
}
