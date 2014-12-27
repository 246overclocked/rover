/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import Libraries.Jaguar246;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.commands.CommandBase;
import edu.wpi.first.wpilibj.templates.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RoverRobot extends IterativeRobot {

    Command autonomousCommand;
    
    public Drivetrain drivetrain;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        getWatchdog().setEnabled(true);
        RobotMap.init();
        CommandBase.init();
        drivetrain = CommandBase.drivetrain;
        SmartDashboard.putBoolean("motorKilled", false);
    }   
    
    public void disabledInit()
    {
        drivetrain.PIDOn(false);
    }
    
    public void disabledPeriodic() {
        allPeriodic();
        
        //Zeros the module angle encoders when the driver presses the button
        if(RobotMap.angleZeroingButton.get())
        {
            drivetrain.zeroAngles();
        }
        if(drivetrain.nav6.isCalibrating()) System.out.println("Calibrating Nav6");
    }

    public void autonomousInit() {
        drivetrain.PIDOn(true);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        allPeriodic();
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
        drivetrain.PIDOn(true);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        allPeriodic();

        //the drivetrain will automatically unwind each of the 4 modules when the robot is not moving
        if(drivetrain.isMoving())
        {
            drivetrain.stopUnwinding();
        }
        else
        {
            drivetrain.unwind();
        }
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        allPeriodic();
        getWatchdog().feed();
        LiveWindow.run();
    }
    
    public void allPeriodic()
    {
        //This is a safety to prevent any of the modules from rotating too far and overtwisting the wires. 
        //If any module angle surpasses RobotMap.UNSAFE_MODULE_ANGLE, the motor controlling it will be automatically shut off
        if(Math.abs(RobotMap.frontModuleEncoder.getDistance()) > RobotMap.UNSAFE_MODULE_ANGLE)
        {
            System.out.println("Stopping front");
            ((Jaguar246)RobotMap.frontWheelMotor).overridingSet(0);
            SmartDashboard.putBoolean("motorKilled", true);
        }
        if(Math.abs(RobotMap.leftModuleEncoder.getDistance()) > RobotMap.UNSAFE_MODULE_ANGLE)
        {
            System.out.println("Stopping left");
            ((Jaguar246)RobotMap.leftWheelMotor).overridingSet(0);
            SmartDashboard.putBoolean("motorKilled", true);
        }
        if(Math.abs(RobotMap.backModuleEncoder.getDistance()) > RobotMap.UNSAFE_MODULE_ANGLE)
        {
            System.out.println("Stopping back");
            ((Jaguar246)RobotMap.backWheelMotor).overridingSet(0);
            SmartDashboard.putBoolean("motorKilled", true);
        }
        if(Math.abs(RobotMap.rightModuleEncoder.getDistance()) > RobotMap.UNSAFE_MODULE_ANGLE)
        {
            System.out.println("Stopping right");
            ((Jaguar246)RobotMap.rightWheelMotor).overridingSet(0);
            SmartDashboard.putBoolean("motorKilled", true);
        }
        //allows the operator to manually return control of all modules to their respective PIDcontrollers
        if(!SmartDashboard.getBoolean("motorKilled", true))
        {
            ((Jaguar246)(RobotMap.frontModuleMotor)).returnControl();
        }
        
        SmartDashboard.putNumber("Heading", drivetrain.nav6.getYaw());
    }
}
