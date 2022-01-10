package frc.lib.motorcontroller;

import frc.lib.encoder.ODN_Encoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface ODN_MotorController {
    /**
     * Set the percentage of maximum power to send to the motor
     * @param speed The speed to set. This should be between -1.0 and 1.0.
     */
    public void set(double speed);

    /**
     * Sets the voltage output of the SpeedController.
     * Compensates for the current bus voltage to ensure that the desired voltage is output
     * even if the battery voltage is below 12V - highly useful when the voltage outputs are "meaningful"
     * (e.g. they come from a feedforward calculation).
     * 
     * NOTE: This function *must* be called regularly in order for voltage compensation to work properly - unlike the ordinary set function, it is not "set it and forget it."
     * @param voltage The voltage to output
     */
    public void setVoltage(double voltage);

    /**
     * Sets the speed of a motor, compensating for any external factors to ensure that the speed of the motor is actually what it is set to be
     * @param speed speed at which to set this motor to move at
     */
    public void setRealSpeed(double speed);

    /**
     * An interface to reverse the direction of a motor controller
     * @param isInverted True is inverted
     */
    public void setInverted(boolean isInverted);

    /**
     * Gets encoder connected to this motor controller
     * @return Encoder object from encoder connected to this motor controller
     */
    public ODN_Encoder getEncoder();

    /**
     * Gets the SpeedController object which is used in this motor controller
     * @return SpeedController object which this motor controller uses
     */
    public MotorController getBackend();
}
