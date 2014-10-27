/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Swerve;

import Libraries.Potentiometer;
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
    double x;
    double y;
    
    double topAbsoluteWheelSpeed;
    
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
    
    public boolean invertSpeed = false;
    
    public SwerveModule(Encoder encoder, Potentiometer modulePot, SpeedController wheelMotor, SpeedController moduleMotor, double topAbsoluteWheelSpeed, double x, double y)
    {
        this.x = x;
        this.y = y;
        
        this.topAbsoluteWheelSpeed = topAbsoluteWheelSpeed;
        
        this.encoder = encoder;
        this.modulePot = modulePot;
        this.wheelMotor = wheelMotor;
        this.moduleMotor = moduleMotor;
        
        speedPID = new PIDController(SPEED_Kp, SPEED_Ki, SPEED_Kd, SPEED_Kf, encoder, wheelMotor, SPEED_PERIOD);
        anglePID = new PIDController(ANGLE_Kp, ANGLE_Ki, ANGLE_Kd, ANGLE_Kf, modulePot, moduleMotor, ANGLE_PERIOD);
        
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
    
    //whenever possible, call setAngle before setSpeed;
    
    // set angle
    public void setAngle(double angle){
        double setPointForward; // angle setpoint if we want the wheel to move forward
        double setPointBackward; // ditto for backwards
        
        
        
        /*
        double bestAngle = angle;
        int startC = (int)((-720 + angle)/180);
        for(int c = startC; c < startC + 8; c++)
        {
            if(Math.abs(angle + 180*c - modulePot.getAverageAngle()) > Math.abs(bestAngle - modulePot.getAverageAngle())) 
            {    
                bestAngle = angle + 180*c;
            }
        }
        if(((bestAngle - angle)/360)%2 == 1) invertSpeed = true;
        anglePID.setSetpoint(bestAngle);
        */
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
}
