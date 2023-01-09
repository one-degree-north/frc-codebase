package frc.lib.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.lib.encoder.ODN_Encoder;
import frc.lib.encoder.ODN_NullEncoder;
import frc.lib.encoder.ODN_CANEncoder;

public class ODN_SparkMax implements ODN_MotorController {

    public static enum MotorType {
        brushed, brushless
    }

    private CANSparkMax m_backend;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private MotorType m_type;

    public ODN_SparkMax(int CAN_ID, MotorType type) {
        m_type = type;
        m_backend = new CANSparkMax(CAN_ID,
                type == MotorType.brushed ? com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
                        : com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        if (m_type == MotorType.brushless) m_encoder = m_backend.getEncoder();
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
        if (m_type == MotorType.brushless) return new ODN_CANEncoder(m_encoder);
        System.err.println("Cannot get encoder from brushed motor, returning null encoder");
        return new ODN_NullEncoder();
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
