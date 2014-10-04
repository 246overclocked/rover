/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package AwfulNameViaPaul;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Paul
 */
public class SwerveModule 
{

    public Encoder encoder;

    public Potentiometer modulePot;

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
    
    public static final int MAX_ROTATIONS = 3; // That is, 3*360 degrees is the maximum the wires will allow the module to spin
    
    public SwerveModule(Encoder encoder, Potentiometer modulePot, SpeedController wheelMotor, SpeedController moduleMotor)
    {
        this.encoder = encoder;
        this.modulePot = modulePot;
        this.wheelMotor = wheelMotor;
        this.moduleMotor = moduleMotor;
        
        speedPID = new PIDController(SPEED_Kp, SPEED_Ki, SPEED_Kd, SPEED_Kf, encoder, wheelMotor, SPEED_PERIOD);
        anglePID = new PIDController(ANGLE_Kp, ANGLE_Ki, ANGLE_Kd, ANGLE_Kf, modulePot, moduleMotor, ANGLE_PERIOD);
        
        speedPID.setAbsoluteTolerance(SPEED_TOLERANCE);
        anglePID.setAbsoluteTolerance(ANGLE_TOLERANCE);
        
        anglePID.setContinuous(true);
        anglePID.setInputRange(0, 360);
        
        speedPID.setOutputRange(-1, 1);
        anglePID.setOutputRange(-1, 1);
    }
    
    // PID Methods
    // set angle
    public void setAngle(double angle){
        anglePID.setSetpoint(angle);
    }
    
    // set wheel speed
    public void setWheelSpeed(double speed){
        speedPID.setSetpoint(speed);
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
    
    // Encoder Methods
    public double getWheelSpeed() {
        return encoder.getRate();
    }
    
    public double getWheelDistance() {
        return encoder.getDistance();
    }
    
    public void resetEncoder(){
        encoder.reset();
    }
    
    // Potentiometer Methods
    public double getPotAverageAngle(){
        return modulePot.getAverageAngle();
    }
    
    public int getNumberOfRotations(){
        return modulePot.getRotation();
    }
    
//    As soon as the angle is greater than 0 degrees, numb of rotations will be either 1 or -1 (depending on direction). 
//    So, + or - 3*360 degrees of rotation would yeild 4 or -4 (depending on direction). When we are at 
//    + or - (MAX_ROTATIONS + 1), which equals + or - 4, we are no longer safe to rotate. As long as we are less 
//    than that, we are safe to rotate.
    public boolean safeToRotate(){
        return (modulePot.getRotation() < MAX_ROTATIONS + 1 || modulePot.getRotation() > -MAX_ROTATIONS - 1);
    }
    
//    When the number of rotations = 3 (counting in the way outlined in the above comment), we have less than 
//    360 degrees of rotation before we are no longer safe to rotate (at which point, the number of rotations would be 4 by our count).
//    Therefore, we want to be warned when we are approaxing the maximum number of rotations.
    public boolean approachingMaxRotations(){
        return (modulePot.getRotation() == MAX_ROTATIONS || modulePot.getRotation() == -MAX_ROTATIONS);
    }
//    Explanation of weird way of counting: When the angle is less than + or - 360 degrees, we want to know which direction
//    we have started to rotate. Therfore, we need to call this either + or - 1 rotations. Zero would not be helpful because
//    we would not be able to distinguish which direction we began to rotate. Up above when defining MAX_ROTATIONS we use the normal "human" way of thinking
//    about the maximum number of rotations so it will be easy for any "average user" to change that value if necessary without thinking
//    about our wierd counting system.
}
