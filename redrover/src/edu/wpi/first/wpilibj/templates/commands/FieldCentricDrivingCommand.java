/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.commands;

/**
 *
 * @author Paul
 */
public abstract class FieldCentricDrivingCommand extends DrivingCommand{
    
    public double updateHeading()
    {
        return -drivetrain.getFieldCentricHeading();
    }
}
