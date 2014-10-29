/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.templates.commands.CommandBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RoverRobot extends IterativeRobot {

    Command autonomousCommand;
    
    NetworkTable diagnosticsTable;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        RobotMap.init();
        // instantiate the command used for the autonomous period
        autonomousCommand = null;

        // Initialize all subsystems
        CommandBase.init();
    }
    
    public void disabledPeriodic() {
        sendDiagnostics();
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
//        autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        sendDiagnostics();
    }

    public void teleopInit() {
	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        sendDiagnostics();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
        sendDiagnostics();
    }
    
    public void sendDiagnostics()
    {
        OI oi = CommandBase.oi;
        diagnosticsTable = NetworkTable.getTable("Diagnostics");
        
        //robot state
        if(isAutonomous()) diagnosticsTable.putString("Robot State", "Autonomous");
        else if(isOperatorControl()) diagnosticsTable.putString("Robot State", "Teleoperated");
        else if(isTest()) diagnosticsTable.putString("Robot State", "Test");
        else diagnosticsTable.putString("Robot State", "Disabled");
        
        //motor outputs
        diagnosticsTable.putNumber("frontLeftWheelMotor", RobotMap.frontLeftWheelMotor.get());
        diagnosticsTable.putNumber("frontRightWheelMotor", RobotMap.frontRightWheelMotor.get());
        diagnosticsTable.putNumber("backLeftWheelMotor", RobotMap.backLeftWheelMotor.get());
        diagnosticsTable.putNumber("backRightWheelMotor", RobotMap.backRightWheelMotor.get());
        diagnosticsTable.putNumber("frontLeftModuleMotor", RobotMap.frontLeftModuleMotor.get());
        diagnosticsTable.putNumber("frontRightModuleMotor", RobotMap.frontRightModuleMotor.get());
        diagnosticsTable.putNumber("backLeftModuleMotor", RobotMap.backLeftModuleMotor.get());
        diagnosticsTable.putNumber("backRightModuleMotor", RobotMap.backRightModuleMotor.get());
        
        //sensor outputs
        diagnosticsTable.putNumber("frontLeftWheelEncoder", RobotMap.frontLeftWheelEncoder.getRate());
        diagnosticsTable.putNumber("frontRightWheelEncoder", RobotMap.frontRightWheelEncoder.getRate());
        diagnosticsTable.putNumber("backLeftWheelEncoder", RobotMap.backLeftWheelEncoder.getRate());
        diagnosticsTable.putNumber("backRightWheelEncoder", RobotMap.backRightWheelEncoder.getRate());
        diagnosticsTable.putNumber("frontLeftModuleEncoder", RobotMap.frontLeftModuleEncoder.getDistance());
        diagnosticsTable.putNumber("frontRightModuleEncoder", RobotMap.frontRightModuleEncoder.getDistance());
        diagnosticsTable.putNumber("backLeftModuleEncoder", RobotMap.backLeftModuleEncoder.getDistance());
        diagnosticsTable.putNumber("backRightModuleEncoder", RobotMap.backRightModuleEncoder.getDistance());
    
        //Joystick values
        for(int i = 1; i <= 6; i++)
        {
            diagnosticsTable.putNumber("driverLeftJoystick Axis " + i, oi.driverLeftJoystick.getRawAxis(i));
            diagnosticsTable.putNumber("driverRightJoystick Axis " + i, oi.driverRightJoystick.getRawAxis(i));
            diagnosticsTable.putNumber("operatorJoystick Axis " + i, oi.operatorJoystick.getRawAxis(i));
        }
        for(int i = 1; i <= 12; i++)
        {
            diagnosticsTable.putBoolean("driverLeftJoystick Button " + i, oi.driverLeftJoystick.getRawButton(i));
            diagnosticsTable.putBoolean("driverRightJoystick Button " + i, oi.driverRightJoystick.getRawButton(i));
            diagnosticsTable.putBoolean("operatorJoystick Button " + i, oi.operatorJoystick.getRawButton(i));
        }
        
        //PID Setpoints
        diagnosticsTable.putNumber("frontLeftModule Speed Setpoint", CommandBase.drivetrain.frontLeftModule.getSpeedSetpoint());
        diagnosticsTable.putNumber("frontLeftModule Angle Setpoint", CommandBase.drivetrain.frontLeftModule.getAngleSetpoint());
        diagnosticsTable.putNumber("frontRightModule Speed Setpoint", CommandBase.drivetrain.frontRightModule.getSpeedSetpoint());
        diagnosticsTable.putNumber("frontRightModule Angle Setpoint", CommandBase.drivetrain.frontRightModule.getAngleSetpoint());
        diagnosticsTable.putNumber("backLeftModule Speed Setpoint", CommandBase.drivetrain.backLeftModule.getSpeedSetpoint());
        diagnosticsTable.putNumber("backLeftModule Angle Setpoint", CommandBase.drivetrain.backLeftModule.getAngleSetpoint());
        diagnosticsTable.putNumber("backRightModule Speed Setpoint", CommandBase.drivetrain.backRightModule.getSpeedSetpoint());
        diagnosticsTable.putNumber("backRightModule Angle Setpoint", CommandBase.drivetrain.backRightModule.getAngleSetpoint());
    }
}
