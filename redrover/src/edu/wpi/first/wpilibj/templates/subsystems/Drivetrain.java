  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;
import Libraries.Vector2D;
import Swerve.SwerveModule;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.Vector;


/**
 *
 * @author michaelsilver
 */
public class Drivetrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;
    
    public Drivetrain()
    {
        frontLeftModule = new SwerveModule(RobotMap.frontLeftWheelEncoder, RobotMap.frontLeftModulePot, RobotMap.frontLeftWheelMotor, RobotMap.frontLeftModuleMotor);
        frontRightModule = new SwerveModule(RobotMap.frontRightWheelEncoder, RobotMap.frontRightModulePot, RobotMap.frontRightWheelMotor, RobotMap.frontRightModuleMotor);
        backLeftModule = new SwerveModule(RobotMap.backLeftWheelEncoder, RobotMap.backLeftModulePot, RobotMap.backLeftWheelMotor, RobotMap.backLeftModuleMotor);
        backRightModule = new SwerveModule(RobotMap.backRightWheelEncoder, RobotMap.backRightModulePot, RobotMap.backRightWheelMotor, RobotMap.backRightModuleMotor);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        
    }
    
//    CRABDRIVE METHODS:
    
//    combines crab and swerve
    public void crabWithTwist(){
        
    }
//    turns all modules in the same direction
    public Vector2D[] crab(double angle, double speed){
        Vector2D[] moduleVectors = new Vector2D[4];
        for(int i=0; i<moduleVectors.length; i++){
            moduleVectors[i] = new Vector2D(false, speed, angle);
        }
        return moduleVectors;
    }
    
//    turns modules tangential to arc
//    where (x-origin, y-origin) is the location of the center of the circle
//    we are turning about
    public void snake(double x-origin, double y-origin){
        
    }
}
