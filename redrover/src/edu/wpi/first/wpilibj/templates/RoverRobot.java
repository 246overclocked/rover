/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import Libraries.Jaguar246;
import Swerve.SwerveModule;
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
    
    public static boolean test1 = false;
    public static boolean test2 = false;
    public static boolean gyroDisabled = false;
    public static boolean gasMode = false;
    
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
        
        //(new Thread(new RoverRobot())).start();
        
        if(test1)
        {
            SmartDashboard.putNumber("speedSetpoint", 0);
            SmartDashboard.putNumber("angleSetpoint", 0);
            SmartDashboard.putBoolean("speedOn", true);
            SmartDashboard.putBoolean("angleOn", true);
            SmartDashboard.putBoolean("unwind", false);
            SmartDashboard.putNumber("speed", 0);
            SmartDashboard.putNumber("angle", 0);
            SmartDashboard.putNumber("K_DELTA", RobotMap.K_MODULE_ANGLE_DELTA);
            SmartDashboard.putNumber("K_TWIST", RobotMap.K_MODULE_ANGLE_TWIST);
            SmartDashboard.putNumber("K_REVERSE", RobotMap.K_MODULE_ANGLE_REVERSE);
        }
        if(test2)
        {
            SmartDashboard.putNumber("speedP", SwerveModule.SPEED_Kp);
            SmartDashboard.putNumber("speedI", SwerveModule.SPEED_Ki);
            SmartDashboard.putNumber("speedD", SwerveModule.SPEED_Kd);
            SmartDashboard.putNumber("speedF", SwerveModule.SPEED_Kf);
            SmartDashboard.putNumber("frontModuleSpeed", 0);
            SmartDashboard.putNumber("leftModuleSpeed", 0);
            SmartDashboard.putNumber("backModuleSpeed", 0);
            SmartDashboard.putNumber("rightModuleSpeed", 0);
            SmartDashboard.putNumber("frontModuleSpeedSetpoint", 0);
            SmartDashboard.putNumber("leftModuleSpeedSetpoint", 0);
            SmartDashboard.putNumber("backModuleSpeedSetpoint", 0);
            SmartDashboard.putNumber("rightModuleSpeedSetpoint", 0);
            SmartDashboard.putNumber("spinRate", 0);
        }
        
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
        if(test2)
        {
            SmartDashboard.putNumber("frontModuleSpeed", drivetrain.frontModule.getWheelSpeed());
            SmartDashboard.putNumber("leftModuleSpeed", drivetrain.leftModule.getWheelSpeed());
            SmartDashboard.putNumber("backModuleSpeed", drivetrain.backModule.getWheelSpeed());
            SmartDashboard.putNumber("rightModuleSpeed", drivetrain.rightModule.getWheelSpeed());
            SmartDashboard.putNumber("frontModuleSpeedSetpoint", drivetrain.frontModule.getSpeedSetpoint());
            SmartDashboard.putNumber("leftModuleSpeedSetpoint", drivetrain.leftModule.getSpeedSetpoint());
            SmartDashboard.putNumber("backModuleSpeedSetpoint", drivetrain.backModule.getSpeedSetpoint());
            SmartDashboard.putNumber("rightModuleSpeedSetpoint", drivetrain.rightModule.getSpeedSetpoint());
        }
        if(test1)
        {
            SwerveModule mod = drivetrain.frontModule;
            
            if(SmartDashboard.getBoolean("unwind", false))
            {
               mod.unwind();
            }
            else
            {
                if(SmartDashboard.getBoolean("speedOn", false))
                {
                    mod.setWheelSpeed(SmartDashboard.getNumber("speedSetpoint", 0));
                }
                else
                {
                    mod.speedPIDOn(false);
                }

                if(SmartDashboard.getBoolean("angleOn", false))
                {
                    mod.setAngle(SmartDashboard.getNumber("angleSetpoint", 0));
                }
                else
                {
                    mod.anglePIDOn(false);
                }
            }
            
            SmartDashboard.putNumber("speed", mod.getWheelSpeed());
            SmartDashboard.putNumber("angle", mod.getModuleAngle());
        }
        else
        {
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
