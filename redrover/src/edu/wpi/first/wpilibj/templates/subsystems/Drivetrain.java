/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 * @author michaelsilver
 */
public class Drivetrain extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    static final double kP = 1;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;
    static final int PERIOD = 20;
    
    public Drivetrain()
    {
        super(kP, kI, kD, PERIOD, kF);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        return 0;
    }

    protected void usePIDOutput(double d) {
        
    }
}
