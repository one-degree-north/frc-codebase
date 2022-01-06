package frc.lib.motorcontroller;

import frc.lib.encoder.Encoder;

import edu.wpi.first.wpilibj.SpeedController;

public interface ODN_MotorController {
    /**
     * Set the speed of a motor as a percentage of the maximum speed
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
     * An interface to reverse the direction of a motor controller
     * @param isInverted True is inverted
     */
    public void setInverted(boolean isInverted);

    /**
     * Gets encoder connected to this motor controller
     * @return Encoder object from encoder connected to this motor controller
     */
    public Encoder getEncoder();

    /**
     * Gets the SpeedController object which is used in this motor controller
     * @return SpeedController object which this motor controller uses
     */
    public SpeedController getBackend();
}
