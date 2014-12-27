/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.commands;

import Libraries.Vector2D;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.templates.RoverRobot;

/**
 *
 * @author michaelsilver
 */
public class CrabWithTwist extends FieldCentricDrivingCommand{

//    driverLeftJoystick is controlling "twist"
//    driverRightJoystick is controlling "crab"
    
    protected Vector2D getCrabVector() {
        return new Vector2D(true, -oi.driverRightJoystick.getX(), oi.driverRightJoystick.getY());
    }

    protected double getSpinRate() {
        if(RoverRobot.test2)
        {
            return SmartDashboard.getNumber("spinRate", 0) / RobotMap.WHEEL_TOP_ABSOLUTE_SPEED;
        }
        return oi.driverLeftJoystick.getX();
    }

    protected Vector2D getCOR() {
        return new Vector2D(true, 0, 0);
    }

    protected void initialize() {
        
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        
    }

    protected void interrupted() {
        end();
    }
    
}
