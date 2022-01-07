package frc.lib.encoder;

public interface ODN_Encoder {
    /**
     * Gets the position of the encoder. By default it is measured in degrees,
     * but units can be changed using {@link ODN_Encoder#setPositionConversionFactor(double)}
     * @return The position of the encoder
     */
    public double getPosition();

    /**
     * Gets the absolute position of the encoder measured in degrees. 
     * Returns a value from 0 to 360 degrees.
     * @return The absolute position of the encoder
     */
    public double getAbsolutePosition();

    /**
     * Sets the position of the encoder. 
     * @param newPosition The new position of the encoder in degrees
     */
    public void setPosition(double newPosition);

    /**
     * Gets the velocity of the encoder. By default it is measured in degrees per second,
     * but units can be changed using {@link ODN_Encoder#setVelocityConversionFactor(double)}
     * @return The velocity of the encoder
     */
    public double getVelocity();

    /**
     * Sets a factor with which to scale all position values from this encoder
     * @param factor The factor by which to scale position values
     */
    public void setPositionConversionFactor(double factor);
    
    /**
     * Sets a factor with which to scale all velocity values from this encoder
     * @param factor The factor by which to scale velocity values
     */
	public void setVelocityConversionFactor(double factor);
}
