/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
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
public class RoverRobot extends IterativeRobot implements Runnable {

    Command autonomousCommand;
    
    public static double startingHeading = 0;
    
    public static boolean gasMode = true;
    
    public Drivetrain drivetrain;
    
    //NetworkTable diagnosticsTable;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

        try 
        {
            RobotMap.init();
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
        }
        
        // instantiate the command used for the autonomous period
        autonomousCommand = null;

        // Initialize all subsystems
        CommandBase.init();
        drivetrain = CommandBase.drivetrain;
        
        //(new Thread(new RoverRobot())).start();
    }   
    
    public void disabledPeriodic() {
        allPeriodic();
        if(RobotMap.angleZeroingButton.get())
        {
            System.out.println("Zeroing encoders");
            drivetrain.zeroAngles();
        }
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
//        autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        allPeriodic();
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        allPeriodic();
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
        LiveWindow.run();
    }
    
    public void allPeriodic()
    {
        
    }
    
    public void run()
    {
        /*
        String[] keys = {
            "time",
            "robotState",
            "frontLeftWheelMotor",
            "frontRightWheelMotor",
            "backLeftWheelMotor",
            "backRightWheelMotor",
            "frontLeftModuleMotor",
            "frontRightModuleMotor",
            "backLeftModuleMotor",
            "backRightModuleMotor",
            "frontLeftWheelEnocder",
            "frontRightWheelEncoder",
            "backLeftWheelEncoder",
            "backRightWheelEncoder",
            "frontLeftModuleEncoder",
            "frontRightModuleEncoder",
            "backLeftModuleEncoder",
            "backRightModuleEncoder",
            "driverLeftJoystickAxis1",
            "driverLeftJoystickAxis2",
            "driverLeftJoystickAxis3",
            "driverLeftJoystickAxis4",
            "driverLeftJoystickAxis5",
            "driverLeftJoystickAxis6",
            "driverRightJoystickAxis1",
            "driverRightJoystickAxis2",
            "driverRightJoystickAxis3",
            "driverRightJoystickAxis4",
            "driverRightJoystickAxis5",
            "driverRightJoystickAxis6",
            "operatorJoystickAxis1",
            "operatorJoystickAxis2",
            "operatorJoystickAxis3",
            "operatorJoystickAxis4",
            "operatorJoystickAxis5",
            "operatorJoystickAxis6",
            "driverLeftJoystickButton1",
            "driverLeftJoystickButton2",
            "driverLeftJoystickButton3",
            "driverLeftJoystickButton4",
            "driverLeftJoystickButton5",
            "driverLeftJoystickButton6",
            "driverLeftJoystickButton7",
            "driverLeftJoystickButton8",
            "driverLeftJoystickButton9",
            "driverLeftJoystickButton10",
            "driverLeftJoystickButton11",
            "driverLeftJoystickButton12",
            "driverRightJoystickButton1",
            "driverRightJoystickButton2",
            "driverRightJoystickButton3",
            "driverRightJoystickButton4",
            "driverRightJoystickButton5",
            "driverRightJoystickButton6",
            "driverRightJoystickButton7",
            "driverRightJoystickButton8",
            "driverRightJoystickButton9",
            "driverRightJoystickButton10",
            "driverRightJoystickButton11",
            "driverRightJoystickButton12",
            "operatorJoystickButton1",
            "operatorJoystickButton2",
            "operatorJoystickButton3",
            "operatorJoystickButton4",
            "operatorJoystickButton5",
            "operatorJoystickButton6",
            "operatorJoystickButton7",
            "operatorJoystickButton8",
            "operatorJoystickButton9",
            "operatorJoystickButton10",
            "operatorJoystickButton11",
            "operatorJoystickButton12",
            "frontLeftModuleSpeedSetpoint",
            "frontLeftModuleAngleSetpoint",
            "frontRightModuleSpeedSetpoint",
            "frontRightModuleAngleSetpoint",
            "backLeftModuleSpeedSetpoint",
            "backLeftModuleAngleSetpoint",
            "backRightModuleSpeedSetpoint",
            "backRightModuleAngleSetpoint"
        };
        
        OI oi = CommandBase.oi;
        diagnosticsTable = NetworkTable.getTable("Diagnostics");
        diagnosticsTable.putString("name", "redRover");
        while(true)
        {
            while(diagnosticsTable.getBoolean("isLogged", false) || Timer.getFPGATimestamp() - RobotMap.DIAGNOSTICS_LOOP_PERIOD < diagnosticsTable.getNumber("time", Integer.MIN_VALUE)) Timer.delay(.001);
            
            //time
            diagnosticsTable.putNumber(keys[0], Timer.getFPGATimestamp());

            //robot state
            if(isAutonomous()) diagnosticsTable.putString(keys[0], "Autonomous");
            else if(isOperatorControl()) diagnosticsTable.putString(keys[0], "Teleoperated");
            else if(isTest()) diagnosticsTable.putString(keys[0], "Test");
            else diagnosticsTable.putString(keys[0], "Disabled");

            //motor outputs
            diagnosticsTable.putNumber(keys[1], RobotMap.frontLeftWheelMotor.get());
            diagnosticsTable.putNumber(keys[2], RobotMap.frontRightWheelMotor.get());
            diagnosticsTable.putNumber(keys[3], RobotMap.backLeftWheelMotor.get());
            diagnosticsTable.putNumber(keys[4], RobotMap.backRightWheelMotor.get());
            diagnosticsTable.putNumber(keys[5], RobotMap.frontLeftModuleMotor.get());
            diagnosticsTable.putNumber(keys[6], RobotMap.frontRightModuleMotor.get());
            diagnosticsTable.putNumber(keys[7], RobotMap.backLeftModuleMotor.get());
            diagnosticsTable.putNumber(keys[8], RobotMap.backRightModuleMotor.get());

            //sensor outputs
            diagnosticsTable.putNumber(keys[9], RobotMap.frontLeftWheelEncoder.getRate());
            diagnosticsTable.putNumber(keys[10], RobotMap.frontRightWheelEncoder.getRate());
            diagnosticsTable.putNumber(keys[11], RobotMap.backLeftWheelEncoder.getRate());
            diagnosticsTable.putNumber(keys[12], RobotMap.backRightWheelEncoder.getRate());
            diagnosticsTable.putNumber(keys[13], RobotMap.frontLeftModuleEncoder.getDistance());
            diagnosticsTable.putNumber(keys[14], RobotMap.frontRightModuleEncoder.getDistance());
            diagnosticsTable.putNumber(keys[15], RobotMap.backLeftModuleEncoder.getDistance());
            diagnosticsTable.putNumber(keys[16], RobotMap.backRightModuleEncoder.getDistance());

            //Joystick values
            for(int i = 1; i <= 6; i++)
            {
                diagnosticsTable.putNumber(keys[16 + i], oi.driverLeftJoystick.getRawAxis(i));
                diagnosticsTable.putNumber(keys[23 + i] + i, oi.driverRightJoystick.getRawAxis(i));
                diagnosticsTable.putNumber(keys[30 + i] + i, oi.operatorJoystick.getRawAxis(i));
            }
            for(int i = 1; i <= 12; i++)
            {
                diagnosticsTable.putBoolean(keys[36 + i], oi.driverLeftJoystick.getRawButton(i));
                diagnosticsTable.putBoolean(keys[48 + i], oi.driverRightJoystick.getRawButton(i));
                diagnosticsTable.putBoolean(keys[60 + i], oi.operatorJoystick.getRawButton(i));
            }

            //PID Setpoints
            diagnosticsTable.putNumber(keys[73], drivetrain.frontLeftModule.getSpeedSetpoint());
            diagnosticsTable.putNumber(keys[74], drivetrain.frontLeftModule.getAngleSetpoint());
            diagnosticsTable.putNumber(keys[75], drivetrain.frontRightModule.getSpeedSetpoint());
            diagnosticsTable.putNumber(keys[76], drivetrain.frontRightModule.getAngleSetpoint());
            diagnosticsTable.putNumber(keys[77], drivetrain.backLeftModule.getSpeedSetpoint());
            diagnosticsTable.putNumber(keys[78], drivetrain.backLeftModule.getAngleSetpoint());
            diagnosticsTable.putNumber(keys[79], drivetrain.backRightModule.getSpeedSetpoint());
            diagnosticsTable.putNumber(keys[80], drivetrain.backRightModule.getAngleSetpoint());
            
            //data confirmation
            diagnosticsTable.putBoolean("isLogged", false);
        }
        */
    }
}
