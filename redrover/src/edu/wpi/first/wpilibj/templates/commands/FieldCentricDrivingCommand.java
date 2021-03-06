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
        //updates the FOV so that it is always pointing true north (the same direction that the driver is facing)
        return drivetrain.getFieldCentricHeading();
    }
}
