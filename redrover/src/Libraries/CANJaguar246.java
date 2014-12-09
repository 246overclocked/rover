/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Libraries;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Paul
 */
public class CANJaguar246 extends CANJaguar{
    
    public CANJaguar246(int deviceNumber) throws CANTimeoutException {
        super(deviceNumber);
    }
    
    public CANJaguar246(int deviceNumber, ControlMode controlMode) throws CANTimeoutException {
        super(deviceNumber, controlMode);
    }
    
    public void setX(double outputValue) throws CANTimeoutException {
        super.setX(outputValue*360);
    }
    
    public void setX(double outputValue, byte syncGroup) throws CANTimeoutException {
        super.setX(outputValue*360, syncGroup);
    }
    
    public double getX() throws CANTimeoutException {
        return super.getX()/360;
    }
}
