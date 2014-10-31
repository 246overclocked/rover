/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.commands;

import Libraries.Vector2D;

/**
 * An abstract subclass of CommandBase used as a basis for all other commands 
 * pertaining to the control scheme of the drivetrain.
 * @author Paul
 */
public abstract class DrivingCommand extends CommandBase {
    
    public DrivingCommand() {
        requires(drivetrain);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        preExecute();
        
        Vector2D crabVector = getCrabVector();
        crabVector.setAngle(crabVector.getAngle() + drivetrain.getFOV());
        Vector2D COR = getCOR();
        COR.setAngle(COR.getAngle() + drivetrain.getFOV());
        
        drivetrain.drive(crabVector.getMagnitude(), crabVector.getAngle(), getSpinRate(), COR.getX(), COR.getY());
        
        postExecute();
    }
    
    // use these to set the parameters of drivetrain.drive(). The results will be automatically adjusted to be relative to the FOV
    protected abstract Vector2D getCrabVector();
    protected abstract double getSpinRate();
    protected abstract Vector2D getCOR();
    
    // methods to modify execute with overwriting current code
    protected void preExecute(){}
    protected void postExecute(){}
}