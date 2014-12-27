/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates.commands;

import Libraries.Vector2D;

/**
 *
 * @author Paul
 */
public class SnakeDrive extends FieldCentricDrivingCommand{

    protected Vector2D getCrabVector() {
        return new Vector2D(true, 0, 0);
    }

    protected double getSpinRate() {
        return oi.driverLeftJoystick.getX();
    }

    protected Vector2D getCOR() {
        return new Vector2D(true, oi.driverRightJoystick.getX(), oi.driverRightJoystick.getY());
    }

    protected void initialize() {}

    protected boolean isFinished() {
        return false;
    }

    protected void end() {}

    protected void interrupted() {
        end();
    }
    
}
