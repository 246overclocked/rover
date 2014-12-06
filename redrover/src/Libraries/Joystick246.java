/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Libraries;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 * @author Paul
 */
public class Joystick246 extends Joystick{
    
    double deadband = .1;

    public Joystick246(final int port) {
        super(port);
    }
    
    public void setDeadband(double deadband)
    {
        this.deadband = deadband;
    }
    
    public double getDeadband()
    {
        return deadband;
    }
    
    /**
     * Get the value of the axis.
     *
     * @param axis The axis to read [1-6].
     * @return The value of the axis.
     */
    public double getRawAxis(final int axis) {
        double val = super.getRawAxis(axis);
        if(Math.abs(val) <= deadband) return 0;
        else return val;
    }
}
