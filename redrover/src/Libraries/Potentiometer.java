package Libraries;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */



import edu.wpi.first.wpilibj.AnalogChannel;

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
     * Get the value of the potentiometer converted to degrees relative to the zeroVoltage
     * 
     * @return the average angle in degrees 
    */
    public double getAngle()
    {
        return (getVoltage() - zeroVoltage) * degreesPerVolt;
    }
    
    /**
     * Get the averaged value of the potentiometer converted to degrees relative to the zeroVoltage
     * 
     * @return the average angle in degrees 
    */
    public double getAverageAngle()
    {
        return (getAverageVoltage() - zeroVoltage) * degreesPerVolt;
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
            getTable().putNumber("Angle", getAverageAngle());
        }
    }
    
}
