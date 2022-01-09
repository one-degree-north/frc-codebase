package frc.lib.motorcontroller;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class ODN_MotorControllerGroup {

    private MotorControllerGroup m_backend;

    public ODN_MotorControllerGroup(ODN_MotorController... controllers) {
        MotorController[] s = new MotorController[controllers.length];
        for(int i=0;i<controllers.length;i++) {
            s[i] = controllers[i].getBackend();
        }
        m_backend = new MotorControllerGroup(s);
    }

    public void set(double speed) {
        m_backend.set(speed);
    }

    public void setInverted(boolean isInverted) {
        m_backend.setInverted(isInverted);
    }

	public MotorController getBackend() {
		return m_backend;
	}

	public void setVoltage(double outputVolts) {
        m_backend.setVoltage(outputVolts);
	}
    
}
