package frc.lib.motorcontroller;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.lib.encoder.ODN_Encoder;

public class ODN_MotorControllerGroup implements ODN_MotorController {

    private MotorControllerGroup m_backend;
    private ODN_Encoder m_encoder;

    public ODN_MotorControllerGroup(ODN_MotorController... controllers) {
        this(controllers[0].getEncoder(), controllers);
    }

    public ODN_MotorControllerGroup(ODN_Encoder encoder, ODN_MotorController... controllers) {
        MotorController[] s = new MotorController[controllers.length];
        for(int i=0;i<controllers.length;i++) {
            s[i] = controllers[i].getBackend();
        }
        m_backend = new MotorControllerGroup(s);
        m_encoder = encoder;
    }

    @Override
    public void set(double speed) {
        m_backend.set(speed);
    }

    @Override
    public ODN_MotorControllerGroup setInverted(boolean isInverted) {
        m_backend.setInverted(isInverted);
        return this;
    }

    @Override
	public MotorController getBackend() {
		return m_backend;
	}

    @Override
	public void setVoltage(double outputVolts) {
        m_backend.setVoltage(outputVolts);
	}

    @Override
    public ODN_Encoder getEncoder() {
        return m_encoder;
    }

    @Override
    public void setRealSpeed(double speed) {
        // TODO: Add exceptions for this case
        // Cannot set real speed for a motor controller group
    }
    
}
