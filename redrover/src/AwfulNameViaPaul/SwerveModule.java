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
import edu.wpi.first.wpilibj.Victor;

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
        
        ((Victor) moduleMotor).setSafetyEnabled(true);
        ((Victor) moduleMotor).setExpiration(.1);
        
        speedPID = new PIDController(SPEED_Kp, SPEED_Ki, SPEED_Kd, SPEED_Kf, encoder, wheelMotor, SPEED_PERIOD);
        anglePID = new PIDController(ANGLE_Kp, ANGLE_Ki, ANGLE_Kd, ANGLE_Kf, modulePot, moduleMotor, ANGLE_PERIOD);
        
        speedPID.setAbsoluteTolerance(SPEED_TOLERANCE);
        anglePID.setAbsoluteTolerance(ANGLE_TOLERANCE);
        
        anglePID.setContinuous();
        anglePID.setInputRange(0, 360);
        
        speedPID.setOutputRange(-1, 1);
        anglePID.setOutputRange(-1, 1);
    }
    
    // PID Methods
    // set angle
    public void setAngle(double angle){
        if(modulePot.getRotation() > MAX_ROTATIONS || modulePot.getRotation() < -MAX_ROTATIONS) //if we have gone past MAX_ROTATIONS. This should never happen because of the code below, but I included it as a fail-safe.
        {
            unwind();
        }
        else if((modulePot.getRotation() == MAX_ROTATIONS && modulePot.getAngle() >= 180) || (modulePot.getRotation() == -MAX_ROTATIONS && modulePot.getAngle() <= 180)) //if we are approaching MAX_ROTATIONS.
        {
            anglePIDOn(true);
            anglePID.setContinuous(false);
        }
        else // if we are safe to rotate
        {
            anglePIDOn(true);
            anglePID.setContinuous();
        }
        anglePID.setSetpoint(angle);
    }
    
    // set wheel speed
    public void setWheelSpeed(double speed){
        speedPID.setSetpoint(speed);
    }
    
    public void unwind() //note that this method should be called repeatedly, not just once.
    {
        if((modulePot.getRotation() == 1 && modulePot.getAngle() < 180) || (modulePot.getRotation() == -1 && modulePot.getAngle() > 180)) //if we are within this range, than setting the setpoint to zero will bring it back to its starting position (0 degrees at 1 rotation)
        {
            anglePIDOn(true);
            anglePID.setSetpoint(0);
        }
        else //if not, then we just have to turn off the PID loop and rotate the modules at full speed towards the starting position (0 degrees at 1 rotation) until we get into that range.
        {
            anglePIDOn(false);
            if(modulePot.getRotation() < 0)
            {
                moduleMotor.set(1);
            }
            else
            {
                moduleMotor.set(-1);
            }
        }
        
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
    
//    Explanation of weird way of counting: When the angle is less than + or - 360 degrees, we want to know which direction
//    we have started to rotate. Therfore, we need to call this either + or - 1 rotations. Zero would not be helpful because
//    we would not be able to distinguish which direction we began to rotate. Up above when defining MAX_ROTATIONS we use the normal "human" way of thinking
//    about the maximum number of rotations so it will be easy for any "average user" to change that value if necessary without thinking
//    about our wierd counting system.
}
