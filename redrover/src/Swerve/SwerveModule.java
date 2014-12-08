/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Swerve;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.templates.RoverRobot;

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
    
    public PIDController speedPID;
    public PIDController anglePID;
    
    public static final double SPEED_Kp = 0.5;
    public static final double SPEED_Ki = 0;
    public static final double SPEED_Kd = 0;
    public static final double SPEED_Kf = 0;
    public static final double SPEED_PERIOD = .1;
    public static final double SPEED_TOLERANCE = 0.25;
    
    public static final double ANGLE_Kp = 0.015; //was .02
    public static final double ANGLE_Ki = 0.0012; //was .008
    public static final double ANGLE_Kd = 0.003; //was 0
    public static final double ANGLE_Kf = 0;
    public static final double ANGLE_PERIOD = .1;
    public static final double ANGLE_TOLERANCE = 0.5;
    
    public boolean invertSpeed = false;
    
    public boolean unwinding = false;
    
    public SwerveModule(Encoder wheelEncoder, Encoder moduleEncoder, SpeedController wheelMotor, SpeedController moduleMotor, double topAbsoluteWheelSpeed, double x, double y, String name)
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
        
        speedPID.setOutputRange(-1, 1);
        anglePID.setOutputRange(-1, 1);
        
        LiveWindow.addSensor("SwerveModule", name + "speedPID", speedPID);
        LiveWindow.addSensor("SwerveModule", name + "anglePID", anglePID);
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

        if(!unwinding)
        {
            final double K_DELTA;
            final double K_TWIST;
            final double K_REVERSE;

            if(RoverRobot.test1)
            {
                K_DELTA = SmartDashboard.getNumber("K_DELTA", RobotMap.K_MODULE_ANGLE_DELTA);
                K_TWIST = SmartDashboard.getNumber("K_TWIST", RobotMap.K_MODULE_ANGLE_TWIST);
                K_REVERSE = SmartDashboard.getNumber("K_REVERSE", RobotMap.K_MODULE_ANGLE_REVERSE);
            }
            else
            {
                K_DELTA = RobotMap.K_MODULE_ANGLE_DELTA;
                K_TWIST = RobotMap.K_MODULE_ANGLE_TWIST;
                K_REVERSE = RobotMap.K_MODULE_ANGLE_REVERSE;
            }

            anglePID.enable();

            angle = angle % 360;

            double setPointForward = angle; // angle setpoint if we want the wheel to move forward
            double setPointBackward = angle + 180; // ditto for backwards

            while(Math.abs(setPointForward - moduleEncoder.getDistance()) > 180
                    && Math.abs(setPointForward) < RobotMap.MAX_MODULE_ANGLE - 180) // while setPointForward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
            {
                if(setPointForward - moduleEncoder.getDistance() < 0) setPointForward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else setPointForward -= 360; //else subtract 360
            }

            while(Math.abs(setPointBackward - moduleEncoder.getDistance()) > 180
                    && Math.abs(setPointBackward) < RobotMap.MAX_MODULE_ANGLE - 180) // while setPointBackward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
            {
                if(setPointBackward - moduleEncoder.getDistance() < 0) setPointBackward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else setPointBackward -= 360; //else subtract 360
            }

            //rate the 2 options based on distance from the current angle, if we will be untwisting the wire, and how fast the wheel is going for setPointBackward to reverse it

            double forwardsRating = 0;
            double backwardsRating = 0;

            //Rating for the distance between where the module is currently pointing and each of the setpoints
            forwardsRating -= K_DELTA*Math.abs(setPointForward - moduleEncoder.getDistance());
            backwardsRating -= K_DELTA*Math.abs(setPointBackward - moduleEncoder.getDistance());

            //Rating boost if this setpoint is closer to the 0 (where the wire is completely untwisted) that the current module angle
            if(setPointForward > 0){
                forwardsRating += (moduleEncoder.getDistance() - setPointForward)*K_TWIST; // positive => we are unwinding (moving closer to zero)
            } else {
                forwardsRating += (setPointForward - moduleEncoder.getDistance())*K_TWIST; // negative => we are winding up (moving farther from zero)
            }

            if(setPointBackward > 0){
                backwardsRating += (moduleEncoder.getDistance() - setPointBackward)*K_TWIST; // positive => we are unwinding (moving closer to zero)
            } else {
                backwardsRating += (setPointBackward - moduleEncoder.getDistance())*K_TWIST; // negative => we are winding up (moving farther from zero)
            }

            //Rating for if the how much the velocity will need to change in order the make the wheel go further. Forwards rating gets a positive boost if wheel is already moving forwards, if the wheel is currently moving backwards it gets a deduction.
            forwardsRating += K_REVERSE * wheelEncoder.getRate();

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
    }
    
    // set wheel speed
    public void setWheelSpeed(double speed){
        if(invertSpeed) speed = -speed;
        if(!RoverRobot.gasMode)
        {
            speedPID.enable();
            speedPID.setSetpoint(speed*topAbsoluteWheelSpeed);
        }
        else
        {
            wheelMotor.set(speed);
        }
    }
    
    public void unwind()
    {
        unwinding = true;
        if(!anglePID.isEnable())anglePID.enable();
        anglePID.setSetpoint(0);
    }
    
    public void stopUnwinding()
    {
        unwinding = false;
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
        moduleEncoder.reset();
    }
}
