/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Swerve;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.templates.RobotMap;

/**
 *
 * @author Paul
 */
public class SwerveModule 
{
    double x;
    double y;
    
    double topAbsoluteWheelSpeed;
    
    public Encoder wheelEncoder;

    public Encoder moduleEncoder;

    public SpeedController wheelMotor;

    public SpeedController moduleMotor;
    
    PIDController speedPID;
    PIDController anglePID;
    
    public static final double SPEED_Kp = 0.5;
    public static final double SPEED_Ki = 0;
    public static final double SPEED_Kd = 0;
    public static final double SPEED_Kf = 0;
    public static final double SPEED_PERIOD = 10;
    public static final double SPEED_TOLERANCE = 0.25;
    
    public static final double ANGLE_Kp = 0.1;
    public static final double ANGLE_Ki = 0;
    public static final double ANGLE_Kd = 0;
    public static final double ANGLE_Kf = 0;
    public static final double ANGLE_PERIOD = 10;
    public static final double ANGLE_TOLERANCE = 0.5;
    
    public boolean invertSpeed = false;
    
    public SwerveModule(Encoder wheelEncoder, Encoder moduleEncoder, SpeedController wheelMotor, SpeedController moduleMotor, double topAbsoluteWheelSpeed, double x, double y)
    {
        this.x = x;
        this.y = y;
        
        this.topAbsoluteWheelSpeed = topAbsoluteWheelSpeed;
        
        this.wheelEncoder = wheelEncoder;
        this.moduleEncoder = moduleEncoder;
        this.wheelMotor = wheelMotor;
        this.moduleMotor = moduleMotor;
        
        speedPID = new PIDController(SPEED_Kp, SPEED_Ki, SPEED_Kd, SPEED_Kf, wheelEncoder, wheelMotor, SPEED_PERIOD);
        anglePID = new PIDController(ANGLE_Kp, ANGLE_Ki, ANGLE_Kd, ANGLE_Kf, moduleEncoder, moduleMotor, ANGLE_PERIOD);
        
        speedPID.setAbsoluteTolerance(SPEED_TOLERANCE);
        anglePID.setAbsoluteTolerance(ANGLE_TOLERANCE);
        
        anglePID.setContinuous();
        anglePID.setInputRange(0, 360);
        
        speedPID.setOutputRange(-1, 1);
        anglePID.setOutputRange(-1, 1);
    }
    
//    coordinates
    public double getX(){
        return x;
    }
    
    public double getY(){
        return y;
    }
    
    // PID Methods
    
    //whenever possible, call setAngle before setSpeed
    
    // set angle
    
    public void setAngle(double angle){
        
        angle = angle % 360;
        
        double setPointForward = angle; // angle setpoint if we want the wheel to move forward
        double setPointBackward = angle + 180; // ditto for backwards
        
        while(Math.abs(setPointForward - moduleEncoder.getDistance()) > 180 
                && setPointForward < RobotMap.MAX_MODULE_ANGLE 
                && setPointForward > -RobotMap.MAX_MODULE_ANGLE) // while setPointForward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
        {
            if(setPointForward - moduleEncoder.getDistance() > 0) setPointForward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
            else setPointForward -= 360; //else subtract 360
        }
        
        while(Math.abs(setPointBackward - moduleEncoder.getDistance()) > 180 
                && setPointBackward < RobotMap.MAX_MODULE_ANGLE 
                && setPointBackward > -RobotMap.MAX_MODULE_ANGLE) // while setPointBackward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
        {
            if(setPointBackward - moduleEncoder.getDistance() > 0) setPointBackward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
            else setPointBackward -= 360; //else subtract 360
        }
        
        //rate the 2 options based on distance from the current angle, if we will be untwisting the wire, and how fast the wheel is going for setPointBackward to reverse it
        
        double forwardsRating = 0;
        double backwardsRating = 0;
        
        forwardsRating -= RobotMap.K_MODULE_ANGLE_DELTA*Math.abs(setPointForward - moduleEncoder.getDistance());
        backwardsRating -= RobotMap.K_MODULE_ANGLE_DELTA*Math.abs(setPointBackward - moduleEncoder.getDistance());
        
        if((setPointForward > 0 && setPointForward < moduleEncoder.getDistance()) || (setPointForward < 0 && setPointForward > moduleEncoder.getDistance())) forwardsRating += RobotMap.K_MODULE_ANGLE_REVERSE;
        if((setPointBackward > 0 && setPointBackward < moduleEncoder.getDistance()) || (setPointBackward < 0 && setPointBackward > moduleEncoder.getDistance())) backwardsRating += RobotMap.K_MODULE_ANGLE_REVERSE;
        
        backwardsRating -= RobotMap.K_MODULE_ANGLE_REVERSE * wheelEncoder.getRate();
        
        if(forwardsRating > backwardsRating) 
        {
            anglePID.setSetpoint(setPointForward);
            invertSpeed = false;
        }
        else
        {
            anglePID.setSetpoint(setPointBackward);
            invertSpeed = true;
        }
    }
    
    // set wheel speed
    public void setWheelSpeed(double speed){
        if(invertSpeed) speed = -speed;
        speedPID.setSetpoint(speed*topAbsoluteWheelSpeed);
    }
    
    public void unwind()
    {
        anglePID.setSetpoint(0);
    }
    
    public void anglePIDOn(boolean on){
        if (on) anglePID.enable();
        else anglePID.disable();
    }
    
    public void speedPIDOn(boolean on){
        if (on) speedPID.enable();
        else speedPID.disable();
    }
    
    public double getAngleSetpoint() {
        return anglePID.getSetpoint();
    }
    
    public double getSpeedSetpoint() {
        return speedPID.getSetpoint();
    }
    
    public boolean angleOnTarget() {
        return anglePID.onTarget();
    }
    
    public boolean speedOnTarget() {
        return speedPID.onTarget();
    }
    
    // Wheel Encoder Methods
    public double getWheelSpeed() {
        return wheelEncoder.getRate();
    }
    
    public double getWheelDistance() {
        return wheelEncoder.getDistance();
    }
    
    public void resetWheelEncoder(){
        wheelEncoder.reset();
    }
    
    // Module Encoder Methods
    
    public double getModuleAngle() {
        return moduleEncoder.getDistance();
    }
    
    public void resetModuleEncoder(){
        wheelEncoder.reset();
    }
}
