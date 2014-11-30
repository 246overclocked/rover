/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.commands;

import Libraries.Vector2D;

/**
 *
 * @author michaelsilver
 */
public class CrabWithAbsoluteTwist extends CrabWithTwist{
    
    protected void initialize() {
        execute();
        
        drivetrain.enableAbsoluteTwist(true);
    }
    
    protected void execute() {
        preExecute();
        
        Vector2D crabVector = getCrabVector();
        crabVector.setAngle(crabVector.getAngle() + drivetrain.getFOV());
        Vector2D COR = getCOR();
        COR.setAngle(COR.getAngle() + drivetrain.getFOV());
        
        drivetrain.driveAbsoluteTwist(crabVector.getMagnitude(), crabVector.getAngle(), oi.driverLeftJoystick.getDirectionDegrees());
        
        postExecute();
    }
    
    protected void end()
    {
        drivetrain.enableAbsoluteTwist(false);
    }
}
