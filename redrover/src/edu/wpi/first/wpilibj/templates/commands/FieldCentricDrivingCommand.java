/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.commands;

import edu.wpi.first.wpilibj.templates.RoverRobot;

/**
 *
 * @author Paul
 */
public abstract class FieldCentricDrivingCommand extends DrivingCommand{
    
    public double updateHeading()
    {
        if(RoverRobot.gyroDisabled)
        {
            return 0;
        }
        else
        {
            return drivetrain.getFieldCentricHeading();
    
        }
    }
}
