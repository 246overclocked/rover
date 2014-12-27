  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;
import Libraries.Vector2D;
import Nav6.BufferingSerialPort;
import Nav6.IMUAdvanced;
import Swerve.SwerveModule;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.templates.RoverRobot;
import edu.wpi.first.wpilibj.templates.commands.CrabWithTwist;
import edu.wpi.first.wpilibj.visa.VisaException;


/**
 * This is the heart of the swerve code. All vector calculations are done here.
 * 
 * @author michaelsilver
 */
public class Drivetrain extends Subsystem {
    
    public SwerveModule frontModule;
    public SwerveModule leftModule;
    public SwerveModule backModule;
    public SwerveModule rightModule;
    
    public double FOV = 0; //the front of the vehicle in degrees. May be used in different ways by different control schemes.
    
    public IMUAdvanced nav6;
    
    public Drivetrain()
    {
        frontModule = new SwerveModule(RobotMap.frontWheelEncoder, RobotMap.frontModuleEncoder, RobotMap.frontWheelMotor, RobotMap.frontModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, 0, RobotMap.FRONT_BACK_LENGTH/2, "frontModule");
        leftModule = new SwerveModule(RobotMap.leftWheelEncoder, RobotMap.leftModuleEncoder, RobotMap.leftWheelMotor, RobotMap.leftModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, -RobotMap.LEFT_RIGHT_WIDTH/2, 0, "leftModule");
        backModule = new SwerveModule(RobotMap.backWheelEncoder, RobotMap.backModuleEncoder, RobotMap.backWheelMotor, RobotMap.backModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, 0, -RobotMap.FRONT_BACK_LENGTH/2, "backModule");
        rightModule = new SwerveModule(RobotMap.rightWheelEncoder, RobotMap.rightModuleEncoder, RobotMap.rightWheelMotor, RobotMap.rightModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, RobotMap.LEFT_RIGHT_WIDTH/2, 0, "rightModule");
        
        //We were having occasional errors with the creation of the nav6 object, so we make 5 attempts before allowing the error to go through and being forced to redeploy.
        int count = 0;
        int maxTries = 5;
        while(true) {
            try {
                nav6 = new IMUAdvanced(new BufferingSerialPort(57600));
                if(nav6 != null) break;
            } catch (VisaException e) {
                if (++count == maxTries)
                {
                    e.printStackTrace();
                    break;
                }
                Timer.delay(.01);
            }
        }
        
        LiveWindow.addSensor("Drivetrain", "Gyro", nav6);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new CrabWithTwist());
        
    }
    
//    CRABDRIVE METHODS:
    
//    turns all modules in the same direction
    public Vector2D[] crab(double angle, double speed){
        Vector2D[] moduleVectors = new Vector2D[4];
        for(int i=0; i<moduleVectors.length; i++){
            moduleVectors[i] = new Vector2D(false, speed, angle);
        }
        return moduleVectors;
    }
    
//    turns modules tangential to arc
//    where (x-cor, y-cor) is the location of the center of the circle
//    we are turning about if we have a cartesian coordinate system with cor
//    being the robot's center of rotation = "origin"
    public Vector2D[] snake(double rate, double corXDist, double corYDist){
        Vector2D cor = new Vector2D(true, corXDist, corYDist);
        Vector2D[] moduleLocations = new Vector2D[4];
        moduleLocations[0] = new Vector2D(true, frontModule.getX(), frontModule.getY());
        moduleLocations[1] = new Vector2D(true, leftModule.getX(), leftModule.getY());
        moduleLocations[2] = new Vector2D(true, backModule.getX(), backModule.getY());
        moduleLocations[3] = new Vector2D(true, rightModule.getX(), rightModule.getY());
        
//        makes the module locations vectors have an origin at the center of rotation
        double[] moduleDists = new double[4]; //array of module distances (the magnitudes of the distance vectors)
        for(int i=0; i<moduleLocations.length; i++){
            moduleLocations[i] = Vector2D.subtractVectors(moduleLocations[i], cor);
            moduleDists[i] = moduleLocations[i].getMagnitude();
        }
        
//        find the farthest module from the center of rotation
        int farthestModule = 0;
        for(int i=0; i<moduleDists.length; i++){
            if(moduleDists[i] > moduleDists[farthestModule]){
                farthestModule = i;
            }
        }
        
//        rotate the moduleLocations vectors -90 degrees.
        Vector2D[] moduleSetpoints = new Vector2D[4];
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = new Vector2D(true, -moduleLocations[i].getY(), moduleLocations[i].getX());
            moduleSetpoints[i].setMagnitude(rate*moduleDists[i]/moduleDists[farthestModule]); //The furthest module should move at the same speed as the rate, and all of the other ones should scale directly porportionally to it based on the ratio of their distances to the center of rotation.
        }
        return moduleSetpoints;
    }
    
    //The primary driving method. Adds the crab and snake vectors together, allowing the robot to drive in any direction while rotating at the same time.
    public void drive(double speed, double direction, double spinRate, double corX, double corY)
    {
        Vector2D[] moduleSetpoints = new Vector2D[4];
        Vector2D[] crab = crab(direction, speed);
        Vector2D[] snake = snake(spinRate, corX, corY);
        
        //Add together the crab and snake vectors. Also find which wheel will be spinning the fastest.
        double largestVector = 0;
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = Vector2D.addVectors(crab[i], snake[i]);
            if(moduleSetpoints[i].getMagnitude() > largestVector) largestVector = moduleSetpoints[i].getMagnitude();
        }
        
        //normalize the vectors so that none of them have a magnitude greater than 1
        if(largestVector > 1)
        {
            for(int i = 0; i < moduleSetpoints.length; i++)
            {
                moduleSetpoints[i].setMagnitude(moduleSetpoints[i].getMagnitude() / largestVector);
            }
        }
        
        frontModule.setAngle(moduleSetpoints[0].getAngle());
        leftModule.setAngle(moduleSetpoints[1].getAngle());
        backModule.setAngle(moduleSetpoints[2].getAngle());
        rightModule.setAngle(moduleSetpoints[3].getAngle());
        
        frontModule.setWheelSpeed(moduleSetpoints[0].getMagnitude());
        leftModule.setWheelSpeed(moduleSetpoints[1].getMagnitude());
        backModule.setWheelSpeed(moduleSetpoints[2].getMagnitude());
        rightModule.setWheelSpeed(moduleSetpoints[3].getMagnitude());        
        
    }
    
    public void setFOV(double fov)
    {
        FOV = fov;
    }
    
    public double getFOV()
    {
        return FOV;
    }
    
    public double getFieldCentricHeading() //returns the direction of the robot relative to the direction the driver is facing.
    {
        return nav6.getYaw();
    }
    
    public boolean isMoving()
    {
        return frontModule.getSpeedSetpoint() != 0
                ||leftModule.getSpeedSetpoint() != 0
                ||backModule.getSpeedSetpoint() != 0
                ||rightModule.getSpeedSetpoint() != 0;
    }
    
    public void PIDOn(boolean on)
    {
        if(on)
        {
            frontModule.speedPIDOn(true);
            leftModule.speedPIDOn(true);
            backModule.speedPIDOn(true);
            rightModule.speedPIDOn(true);
            
            frontModule.anglePIDOn(true);
            leftModule.anglePIDOn(true);
            backModule.anglePIDOn(true);
            rightModule.anglePIDOn(true);
        }
        else
        {
            frontModule.speedPIDOn(false);
            leftModule.speedPIDOn(false);
            backModule.speedPIDOn(false);
            rightModule.speedPIDOn(false);
            
            frontModule.anglePIDOn(false);
            leftModule.anglePIDOn(false);
            backModule.anglePIDOn(false);
            rightModule.anglePIDOn(false);
        }
    }
    
    public void unwind()
    {
        frontModule.unwind();
        leftModule.unwind();
        backModule.unwind();
        rightModule.unwind();
    }
    
    public void stopUnwinding()
    {
        frontModule.stopUnwinding();
        leftModule.stopUnwinding();
        backModule.stopUnwinding();
        rightModule.stopUnwinding();
    }
    
    public void zeroAngles()
    {
        frontModule.resetModuleEncoder();
        leftModule.resetModuleEncoder();
        backModule.resetModuleEncoder();
        rightModule.resetModuleEncoder();
    }
     
    public boolean isOverRotated()
    {
        return Math.abs(frontModule.getModuleAngle()) > RobotMap.MAX_MODULE_ANGLE 
                || Math.abs(leftModule.getModuleAngle()) > RobotMap.MAX_MODULE_ANGLE 
                || Math.abs(backModule.getModuleAngle()) > RobotMap.MAX_MODULE_ANGLE 
                || Math.abs(rightModule.getModuleAngle()) > RobotMap.MAX_MODULE_ANGLE;
    }
}
