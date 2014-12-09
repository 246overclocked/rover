/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Swerve;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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

    public SpeedController wheelMotor;

    public CANJaguar moduleMotor;
    
    public PIDController speedPID;
    
    public static final double SPEED_Kp = 0.5;
    public static final double SPEED_Ki = 0;
    public static final double SPEED_Kd = 0;
    public static final double SPEED_Kf = 0;
    public static final double SPEED_PERIOD = .1;
    public static final double SPEED_TOLERANCE = 0.25;
    
    public int moduleEncoderZeroPosition = 0;
    
    public boolean invertSpeed = false;
    
    public boolean unwinding = false;
    
    public SwerveModule(Encoder wheelEncoder, Encoder moduleEncoder, SpeedController wheelMotor, SpeedController moduleMotor, double topAbsoluteWheelSpeed, double x, double y, String name)
    {
        this.x = x;
        this.y = y;
        
        this.topAbsoluteWheelSpeed = topAbsoluteWheelSpeed;
        
        this.wheelEncoder = wheelEncoder;
        this.wheelMotor = wheelMotor;
        this.moduleMotor = (CANJaguar)moduleMotor;
        
        speedPID = new PIDController(SPEED_Kp, SPEED_Ki, SPEED_Kd, SPEED_Kf, wheelEncoder, wheelMotor, SPEED_PERIOD);
        
        speedPID.setAbsoluteTolerance(SPEED_TOLERANCE);
        
        speedPID.setOutputRange(-1, 1);
        
        LiveWindow.addSensor("SwerveModule", name + "speedPID", speedPID);
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
    
    public void setAngle(double angle) {

        if(!unwinding)
        {
            final double K_DELTA = RobotMap.K_MODULE_ANGLE_DELTA;
            final double K_TWIST = RobotMap.K_MODULE_ANGLE_TWIST;
            final double K_REVERSE = RobotMap.K_MODULE_ANGLE_REVERSE;

            angle = angle % 360;
            
            double currentAngle;
            try
            {
                currentAngle = moduleMotor.getPosition();
            }
            catch(CANTimeoutException e)
            {
                e.printStackTrace();
                return;
            }

            double setPointForward = angle; // angle setpoint if we want the wheel to move forward
            double setPointBackward = angle + 180; // ditto for backwards

            while(Math.abs(setPointForward - currentAngle) > 180
                    && Math.abs(setPointForward) < RobotMap.MAX_MODULE_ANGLE - 180) // while setPointForward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
            {
                if(setPointForward - currentAngle < 0) setPointForward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else setPointForward -= 360; //else subtract 360
            }

            while(Math.abs(setPointBackward - currentAngle) > 180
                    && Math.abs(setPointBackward) < RobotMap.MAX_MODULE_ANGLE - 180) // while setPointBackward is not the closest possible angle to moduleEncoder and getting closer would not bring it past MAX_MOUDLE_ROTATIONS
            {
                if(setPointBackward - currentAngle < 0) setPointBackward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else setPointBackward -= 360; //else subtract 360
            }

            //rate the 2 options based on distance from the current angle, if we will be untwisting the wire, and how fast the wheel is going for setPointBackward to reverse it

            double forwardsRating = 0;
            double backwardsRating = 0;

            //Rating for the distance between where the module is currently pointing and each of the setpoints
            forwardsRating -= K_DELTA*Math.abs(setPointForward - currentAngle);
            backwardsRating -= K_DELTA*Math.abs(setPointBackward - currentAngle);

            //Rating boost if this setpoint is closer to the 0 (where the wire is completely untwisted) that the current module angle
            if(setPointForward > 0){
                forwardsRating += (currentAngle - setPointForward)*K_TWIST; // positive => we are unwinding (moving closer to zero)
            } else {
                forwardsRating += (setPointForward - currentAngle)*K_TWIST; // negative => we are winding up (moving farther from zero)
            }

            if(setPointBackward > 0){
                backwardsRating += (currentAngle - setPointBackward)*K_TWIST; // positive => we are unwinding (moving closer to zero)
            } else {
                backwardsRating += (setPointBackward - currentAngle)*K_TWIST; // negative => we are winding up (moving farther from zero)
            }

            //Rating for if the how much the velocity will need to change in order the make the wheel go further. Forwards rating gets a positive boost if wheel is already moving forwards, if the wheel is currently moving backwards it gets a deduction.
            forwardsRating += K_REVERSE * wheelEncoder.getRate();

            if(forwardsRating > backwardsRating)
            {
                try
                {
                    moduleMotor.setX(setPointForward);
                    invertSpeed = false; 
                }
                catch(CANTimeoutException e)
                {
                    e.printStackTrace();
                }
            }
            else
            {
                try
                {
                    moduleMotor.setX(setPointBackward);
                    invertSpeed = true;
                }
                catch(CANTimeoutException e)
                {
                    e.printStackTrace();
                }
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
        try
        {
            moduleMotor.setX(0);
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
        }
    }
    
    public void stopUnwinding()
    {
        unwinding = false;
    }
    
    public void speedPIDOn(boolean on){
        if (on) speedPID.enable();
        else speedPID.disable();
    }
    
    public double getAngleSetpoint() {
        try
        {
            return moduleMotor.getX();
        }
        catch(CANTimeoutException e)
        {
              e.printStackTrace();
              return 0;
        }
    }
    
    public double getSpeedSetpoint() {
        return speedPID.getSetpoint();
    }
    
    public boolean angleOnTarget() {
        try
        {
            return Math.abs(moduleMotor.getX() - moduleMotor.getPosition()) > 3;
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
            return false;
        }
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
        try
        {
            return moduleMotor.getPosition();
    
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
            return 0;
        }
    }
    
    public void resetModuleEncoder(){
        try
        {
            moduleMotor.enableControl(moduleMotor.getPosition());
    
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
        }
    }
}
