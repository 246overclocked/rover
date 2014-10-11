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
        frontLeftModule = new SwerveModule(RobotMap.frontLeftWheelEncoder, RobotMap.frontLeftModulePot, RobotMap.frontLeftWheelMotor, RobotMap.frontLeftModuleMotor, -RobotMap.LEFTRIGHTWIDTH/2, RobotMap.FRONTBACKLENGTH/2);
        frontRightModule = new SwerveModule(RobotMap.frontRightWheelEncoder, RobotMap.frontRightModulePot, RobotMap.frontRightWheelMotor, RobotMap.frontRightModuleMotor, RobotMap.LEFTRIGHTWIDTH/2, RobotMap.FRONTBACKLENGTH/2);
        backLeftModule = new SwerveModule(RobotMap.backLeftWheelEncoder, RobotMap.backLeftModulePot, RobotMap.backLeftWheelMotor, RobotMap.backLeftModuleMotor, -RobotMap.LEFTRIGHTWIDTH/2, -RobotMap.FRONTBACKLENGTH/2);
        backRightModule = new SwerveModule(RobotMap.backRightWheelEncoder, RobotMap.backRightModulePot, RobotMap.backRightWheelMotor, RobotMap.backRightModuleMotor, RobotMap.LEFTRIGHTWIDTH/2, -RobotMap.FRONTBACKLENGTH/2);
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
//    we are turning about if we have a cartesian coordinate system with the
//    robot's center as the origin
    public Vector2D[] snake(double rate, double originXDist, double orignYDist){
        Vector2D origin = new Vector2D(true, originXDist, orignYDist);
        Vector2D[] moduleLocations = new Vector2D[4];
        moduleLocations[0] = new Vector2D(true, frontLeftModule.getX(), frontLeftModule.getY());
        moduleLocations[1] = new Vector2D(true, frontRightModule.getX(), frontRightModule.getY());
        moduleLocations[2] = new Vector2D(true, backLeftModule.getX(), backLeftModule.getY());
        moduleLocations[3] = new Vector2D(true, backRightModule.getX(), backRightModule.getY());
        
//        makes the module locations relative to the origin of rotation
        for(int i=0; i<moduleLocations.length; i++){
            moduleLocations[i] = Vector2D.subtractVectors(moduleLocations[i], origin);
        }
        
        Vector2D[] moduleSetpoints = new Vector2D[4];
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = new Vector2D(true, -moduleLocations[i].unitVector().getY(), moduleLocations[i].unitVector().getX());
            moduleSetpoints[i].setMagnitude(rate); // don't know if positive rate will spin clock or counterclock -- TODO: need to experiment
        }
        return moduleSetpoints;
    }
}
