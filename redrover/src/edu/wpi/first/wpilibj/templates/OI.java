
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.DigitalIOButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.templates.commands.CommandBase;
import edu.wpi.first.wpilibj.templates.commands.Unwind;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    
    public Joystick driverLeftJoystick;
    public Joystick driverRightJoystick;
    public Joystick operatorJoystick;
    
    public OI()
    {
        driverLeftJoystick = new Joystick(1);
        driverRightJoystick = new Joystick(2);
        operatorJoystick = new Joystick(3);
        
        new Button() {

            public boolean get() {
                return !CommandBase.drivetrain.isMoving();
            }
        }.whenPressed(new Unwind());
    }
}

