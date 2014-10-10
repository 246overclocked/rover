/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Libraries;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.SensorBase;

/**
 *
 * @author Paul
 */
public class Potentiometer extends AnalogChannel
{
    
    public double degreesPerVolt = 1;
    public double zeroVoltage = 2.5;
    
     /**
     * Construct a potentiometer on the default module.
     *
     * @param channel The channel number to represent.
     */
    public Potentiometer(int channel)
    {
        super(channel);
    }
    
     /**
     * Construct a potentiometer on a specified module.
     *
     * @param moduleNumber The digital module to use (1 or 2).
     * @param channel The channel number to represent.
     */
    public Potentiometer(int moduleNumber, int channel)
    {
        super(moduleNumber, channel);
    }
    
    /**
     * Construct a potentiometer on the default module.
     *
     * @param channel The channel number to represent.
     * @param degreesPerVolt The conversion rate to go from voltage to degrees.
     * @param zeroVoltage The voltage that should be considered an angle of 0 with 1 rotation.
     */
    public Potentiometer(int channel, double degreesPerVolt, double zeroVoltage)
    {
        this(getDefaultAnalogModule(), channel, degreesPerVolt, zeroVoltage); 
    }
    
     /**
     * Construct a potentiometer on a specified module.
     *
     * @param moduleNumber The digital module to use (1 or 2).
     * @param channel The channel number to represent.
     * @param degreesPerVolt The conversion rate to go from voltage to degrees.
     * @param zeroVoltage The voltage that should be considered an angle of 0 with 1 rotation.
     */
    public Potentiometer(int moduleNumber, int channel, double degreesPerVolt, double zeroVoltage)
    {
        super(moduleNumber, channel);
        this.degreesPerVolt = degreesPerVolt;
        this.zeroVoltage = zeroVoltage;
    }
    
    /**
     * Set the degreesPerVolt and zeroVoltage
     * @param degreesPerVolt The conversion rate to go from voltage to degrees.
     * @param zeroVoltage The voltage that should be considered an angle of 0 with 1 rotation.
     */
    public void setConversions(double degreesPerVolt, double zeroVoltage)
    {
        this.degreesPerVolt = degreesPerVolt;
        this.zeroVoltage = zeroVoltage;
    }
    
    /**
     * Get the degreesPerVolt
     * @return The conversion rate to go from voltage to degrees.
     */
    public double getDegreesPerVolt()
    {
        return degreesPerVolt;
    }
    
    /**
     * Get the zeroVoltage
     * @return The voltage that should be considered an angle of 0 with 1 rotation.
     */
    public double getZeroVoltage()
    {
        return zeroVoltage;
    }
    
    /**
     * Get the value of the potentiometer converted to degrees between 0 and 360 relative to the zeroVoltage
     * @return the angle in degrees 
    */
    public double getAngle()
    {
        double angle = (getVoltage() - zeroVoltage) * degreesPerVolt;
        angle = angle % 360;
        if(angle < 0) angle = 360 + angle;
        return angle;
    }
    
    /**
     * Get the average value of the potentiometer converted to degrees between 0 and 360 relative to the zeroVoltage
     * 
     * @return the average angle in degrees 
    */
    public double getAverageAngle()
    {
        double angle = (getAverageVoltage() - zeroVoltage) * degreesPerVolt;
        angle = angle % 360;
        if(angle < 0) angle = 360 + angle;
        return angle;
    }
    
    /**
     * Get the current number of full rotations past the zeroVoltage.
     * A voltage slightly larger than or equal to the zeroVoltage will return 1.
     * A voltage slightly smaller than the zeroVoltage will return -1.
     * @return the number of full rotations from the zeroVoltage.
     */
    public int getRotation()
    {
        double angle = (getVoltage() - zeroVoltage) * degreesPerVolt;
        int rotation = (int) angle/360;
        if(angle < 0) rotation--;
        else rotation++;
        return rotation;
    }
    
     /**
     * Get the average angle value for use with PIDController.
     * @return the average value
     */
    public double pidGet() 
    {
        return getAverageAngle();
    }
    
    /**
     * {@inheritDoc}
     */
    public void updateTable() 
    {
        if (getTable() != null) {
            getTable().putNumber("Angle", getAverageVoltage());
            getTable().putNumber("Rotation", getRotation());
        }
    }
    
}
