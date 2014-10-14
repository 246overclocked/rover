
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.DigitalIOButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    
    public Joystick joystick2D;
    public Joystick joystick3D;
    public Joystick operatorJoystick;
    
    public OI()
    {
        joystick2D = new Joystick(1);
        joystick3D = new Joystick(2);
        operatorJoystick = new Joystick(3);
    }
    
    public double get2DX()
    {
        return joystick2D.getRawAxis(1);
    }
    public double get2DY()
    {
        return joystick2D.getRawAxis(2);
    }
    public double get3DX()
    {
        return joystick2D.getRawAxis(1);
    }
    public double get3DY()
    {
        return joystick2D.getRawAxis(2);
    }
    public double get3DTwist()
    {
        return joystick2D.getRawAxis(3);
    }
}

