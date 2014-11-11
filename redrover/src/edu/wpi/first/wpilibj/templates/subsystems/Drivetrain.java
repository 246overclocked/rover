  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;
import Libraries.Vector2D;
import Swerve.SwerveModule;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 * @author michaelsilver
 */
public class Drivetrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    public SwerveModule frontLeftModule;
    public SwerveModule frontRightModule;
    public SwerveModule backLeftModule;
    public SwerveModule backRightModule;
    
    public double FOV = 0; //the front of the vehicle in degrees. May be used in different ways by different control schemes.
    
    public Drivetrain()
    {
        frontLeftModule = new SwerveModule(RobotMap.frontLeftWheelEncoder, RobotMap.frontLeftModuleEncoder, RobotMap.frontLeftWheelMotor, RobotMap.frontLeftModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, -RobotMap.LEFT_RIGHT_WIDTH/2, RobotMap.FRONT_BACK_LENGTH/2);
        frontRightModule = new SwerveModule(RobotMap.frontRightWheelEncoder, RobotMap.frontRightModuleEncoder, RobotMap.frontRightWheelMotor, RobotMap.frontRightModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, RobotMap.LEFT_RIGHT_WIDTH/2, RobotMap.FRONT_BACK_LENGTH/2);
        backLeftModule = new SwerveModule(RobotMap.backLeftWheelEncoder, RobotMap.backLeftModuleEncoder, RobotMap.backLeftWheelMotor, RobotMap.backLeftModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, -RobotMap.LEFT_RIGHT_WIDTH/2, -RobotMap.FRONT_BACK_LENGTH/2);
        backRightModule = new SwerveModule(RobotMap.backRightWheelEncoder, RobotMap.backRightModuleEncoder, RobotMap.backRightWheelMotor, RobotMap.backRightModuleMotor, RobotMap.WHEEL_TOP_ABSOLUTE_SPEED, RobotMap.LEFT_RIGHT_WIDTH/2, -RobotMap.FRONT_BACK_LENGTH/2);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        
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
        moduleLocations[0] = new Vector2D(true, frontLeftModule.getX(), frontLeftModule.getY());
        moduleLocations[1] = new Vector2D(true, frontRightModule.getX(), frontRightModule.getY());
        moduleLocations[2] = new Vector2D(true, backLeftModule.getX(), backLeftModule.getY());
        moduleLocations[3] = new Vector2D(true, backRightModule.getX(), backRightModule.getY());
        
//        makes the module locations relative to the center of rotation
        double[] moduleDists = new double[4]; //array of module distances (the magnitudes of the distance vectors)
        for(int i=0; i<moduleLocations.length; i++){
            moduleLocations[i] = Vector2D.subtractVectors(moduleLocations[i], cor);
            moduleDists[i] = moduleLocations[i].getMagnitude();
        }
        
//        find the farthest module
        int farthestModule = 0;
        for(int i=0; i<moduleDists.length; i++){
            if(moduleDists[i] > moduleDists[farthestModule]){
                farthestModule = i;
            }
        }
        
        Vector2D[] moduleSetpoints = new Vector2D[4];
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = new Vector2D(true, -moduleLocations[i].unitVector().getY(), moduleLocations[i].unitVector().getX());
            moduleSetpoints[i].setMagnitude(rate*moduleDists[i]/moduleDists[farthestModule]); //set all modules' rate according to their distance from COR
        }
        return moduleSetpoints;
    }
    
    // direction will be continuously updated -- don't worry about changing it with gyro here
    //everything in this method is robotcentric.
    public void drive(double speed, double direction, double spinRate, double corX, double corY)
    {
        Vector2D[] moduleSetpoints = new Vector2D[4];
        Vector2D[] crab = crab(direction, speed);
        Vector2D[] snake = snake(spinRate, corX, corY);
        
        double largestVector = 0;
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = Vector2D.addVectors(crab[i], snake[i]);
            if(moduleSetpoints[i].getMagnitude() > largestVector) largestVector = moduleSetpoints[i].getMagnitude();
        }
        
        if(largestVector > 1)
        {
            //normalize the vectors so that none of them have a magnitude greater than 1
            for(int i = 0; i < moduleSetpoints.length; i++)
            {
                moduleSetpoints[i].setMagnitude(moduleSetpoints[i].getMagnitude() / largestVector);
            }
        }
        
        frontLeftModule.setAngle(moduleSetpoints[0].getAngle());
        frontRightModule.setAngle(moduleSetpoints[1].getAngle());
        backLeftModule.setAngle(moduleSetpoints[2].getAngle());
        backRightModule.setAngle(moduleSetpoints[3].getAngle());
        
        frontLeftModule.setWheelSpeed(moduleSetpoints[0].getMagnitude());
        frontRightModule.setWheelSpeed(moduleSetpoints[1].getMagnitude());
        backLeftModule.setWheelSpeed(moduleSetpoints[2].getMagnitude());
        backRightModule.setWheelSpeed(moduleSetpoints[3].getMagnitude());        
        
    }
    
    //Code for turning the robot to a certain angle relative to the field
    
    public static final double ABSOLUTE_TWIST_kP = 1;
    public static final double ABSOLUTE_TWIST_kI = 0;
    public static final double ABSOLUTE_TWIST_kD = 0;
    public static final double ABSOLUTE_TWIST_kF = 0;
    public static final double ABSOLUTE_TWIST_PERIOD = 20;
    public AbsoluteTwistPIDOutput absoluteTwistPIDOutput = new AbsoluteTwistPIDOutput();
    
    PIDController absoluteTwistPID = new PIDController(ABSOLUTE_TWIST_kP, ABSOLUTE_TWIST_kI, ABSOLUTE_TWIST_kD, ABSOLUTE_TWIST_kF, /* TODO: Set source */ null, absoluteTwistPIDOutput, ABSOLUTE_TWIST_PERIOD);
    
    /**
     *@author Paul Terrasi
     */
    public class AbsoluteTwistPIDOutput implements PIDOutput
    {
        public double speed = 0;
        public double direction = 0;
        
        public void pidWrite(double output) {
            drive(speed, direction, output, 0, 0);
        }

    }
    
    public void driveAbsoluteTwist(double speed, double direction, double absoluteAngle){
        absoluteTwistPIDOutput.speed = speed;
        absoluteTwistPIDOutput.direction = direction;
        absoluteTwistPID.setSetpoint(absoluteAngle);
    }
    
    public void enableAbsoluteTwist(boolean on) {
        if(on) absoluteTwistPID.enable();
        else absoluteTwistPID.disable();
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- congratulations you did it, your prize is the smiley face just to the right----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:( hahaha
    
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
        //TODO: Code this method.
        return 0;
    }
    
    public boolean isMoving()
    {
        return frontLeftModule.getSpeedSetpoint() != 0
                ||frontRightModule.getSpeedSetpoint() != 0
                ||backLeftModule.getSpeedSetpoint() != 0
                ||backRightModule.getSpeedSetpoint() != 0;
    }
    
    public void unwind()
    {
        frontLeftModule.unwind();
        frontRightModule.unwind();
        backLeftModule.unwind();
        backRightModule.unwind();
    }
    
    public void zeroAngles()
    {
        frontLeftModule.resetModuleEncoder();
        frontRightModule.resetModuleEncoder();
        backLeftModule.resetModuleEncoder();
        backRightModule.resetModuleEncoder();
    }
            
}
