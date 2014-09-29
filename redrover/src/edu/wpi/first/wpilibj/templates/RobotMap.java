package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // Drivetrain constructors  
    // sensors
        // wheel speed 
    public static Encoder frontLeftWheelEncoder;
    public static Encoder frontRightWheelEncoder;
    public static Encoder backLeftWheelEncoder;
    public static Encoder backRightWheelEncoder;
        // wheel angle direction 
    public static AnalogChannel frontLeftModulePot;
    public static AnalogChannel frontRightModulePot;
    public static AnalogChannel backLeftModulePot;
    public static AnalogChannel backRightModulePot;
    
    // motors
    public static SpeedController frontLeftWheelMotor;
    public static SpeedController frontRightWheelMotor;
    public static SpeedController backLeftWheelMotor;
    public static SpeedController backRightWheelMotor;
    
//    by "module" we mean angle control of the wheel
    public static SpeedController frontLeftModuleMotor;
    public static SpeedController frontRightModuleMotor;
    public static SpeedController backLeftModuleMotor;
    public static SpeedController backRightModuleMotor;
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static final int leftMotor = 1;
    // public static final int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static final int rangefinderPort = 1;
    // public static final int rangefinderModule = 1;
    
    public static void init() {
        frontLeftWheelEncoder = new Encoder(1, 2, false);
        frontRightWheelEncoder = new Encoder(3, 4, false);
        backLeftWheelEncoder = new Encoder(5, 6, false);
        backRightWheelEncoder = new Encoder(7, 8, false);
        
        frontLeftModulePot = new AnalogChannel(9);
        frontRightModulePot = new AnalogChannel(10);
        backLeftModulePot = new AnalogChannel(11);
        backRightModulePot = new AnalogChannel(12);
            
        frontLeftWheelMotor = new Victor(1);
        frontRightWheelMotor = new Victor(2);
        backLeftWheelMotor = new Victor(3);
        backRightWheelMotor = new Victor(4);
        
        frontLeftModuleMotor = new Victor(5);
        frontRightModuleMotor = new Victor(6);
        backLeftModuleMotor = new Victor(7);
        backRightModuleMotor = new Victor(8);
    }
}
