package frc.lib.motorcontroller;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class ODN_MotorControllerGroup {

    private SpeedControllerGroup m_backend;

    public ODN_MotorControllerGroup(ODN_MotorController... controllers) {
        SpeedController[] s = new SpeedController[controllers.length];
        for(int i=0;i<controllers.length;i++) {
            s[i] = controllers[i].getBackend();
        }
        m_backend = new SpeedControllerGroup(s);
    }

    public void set(double speed) {
        m_backend.set(speed);
    }

    public void setInverted(boolean isInverted) {
        m_backend.setInverted(isInverted);
    }

	public SpeedController getBackend() {
		return m_backend;
	}

	public void setVoltage(double outputVolts) {
        m_backend.setVoltage(outputVolts);
	}
    
}
