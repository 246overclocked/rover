/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Libraries;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author Paul
 */
public class Victor246 extends Victor {
    
    boolean overridden = false;
    
    public Victor246(final int channel)
    {
        super(channel);
    }
    
    public Victor246(final int slot, final int channel) {
        super(slot, channel);
    }
    
    public void set(double speed)
    {
        if(!overridden) super.set(speed);
    }
    
    /**
     * sets the speed of the motor, and disables the set command
     * @param speed the speed which the motor should move at
     */
    public void overridingSet(double speed)
    {
        overridden = true;
        super.set(speed);
    }
    
    /**
     * enables the set command after overridingSet has been called
     */
    public void returnControl()
    {
        overridden = false;
    }
}
