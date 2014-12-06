
package edu.wpi.first.wpilibj.templates;

import Libraries.Joystick246;
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
    
    public Joystick246 driverLeftJoystick;
    public Joystick246 driverRightJoystick;
    public Joystick246 operatorJoystick;
    
    public OI()
    {
        driverLeftJoystick = new Joystick246(1);
        driverLeftJoystick.setDeadband(.1);
        driverRightJoystick = new Joystick246(2);
        driverRightJoystick.setDeadband(.1);
        operatorJoystick = new Joystick246(3);
        operatorJoystick.setDeadband(.1);
        
        /*
        new Button() {

            public boolean get() {
                System.out.println(CommandBase.drivetrain.isMoving());
                return !CommandBase.drivetrain.isMoving();
            }
        }.whenPressed(new Unwind());
        */
        (new JoystickButton(driverLeftJoystick, 3)).whileActive(new Unwind());
    }
}

