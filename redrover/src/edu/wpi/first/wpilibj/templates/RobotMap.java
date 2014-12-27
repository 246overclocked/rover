package edu.wpi.first.wpilibj.templates;

import Libraries.Jaguar246;
import Libraries.Victor246;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    
//    Dimentions
    //in a standard unit of measure -- we must be consistent throughout
//    distance between "left" and "right" modules
    public static final double LEFT_RIGHT_WIDTH = 25; // 12in = 1ft
//    distance between "front" and "back" modules
    public static final double FRONT_BACK_LENGTH = 25;
//    it is assumed that the robot's center is located smack halfway between the
//    "front" and "back" modules and the "left" and "right" modules
    public static final double WHEEL_TOP_ABSOLUTE_SPEED = 11;
    
    //the constants for optimizing which direction the swerveModules will turn
    public static final double K_MODULE_ANGLE_DELTA = 1;
    public static final double K_MODULE_ANGLE_TWIST = 0;
    public static final double K_MODULE_ANGLE_REVERSE = 0;
    
    public static final double MAX_MODULE_ANGLE = 1*360 + 180;
    public static final double UNSAFE_MODULE_ANGLE = MAX_MODULE_ANGLE + 360;
    
    public static final double DIAGNOSTICS_LOOP_PERIOD = 0.02; //in seconds
    
    public static final double WHEEL_ENCODER_DISTANCE_PER_TICK = .00628;
    public static final double MODULE_ENCODER_DISTANCE_PER_TICK = .603;
    
// Drivetrain constructors  
    // sensors
        // wheel speed 
    public static Encoder frontWheelEncoder;
    public static Encoder leftWheelEncoder;
    public static Encoder backWheelEncoder;
    public static Encoder rightWheelEncoder;
        // wheel angle direction 
    public static Encoder frontModuleEncoder;
    public static Encoder leftModuleEncoder;
    public static Encoder backModuleEncoder;
    public static Encoder rightModuleEncoder;
    
    // motors
    public static SpeedController frontWheelMotor;
    public static SpeedController leftWheelMotor;
    public static SpeedController backWheelMotor;
    public static SpeedController rightWheelMotor;
    
//    by "module" we mean angle control of the wheel
    public static SpeedController frontModuleMotor;
    public static SpeedController leftModuleMotor;
    public static SpeedController backModuleMotor;
    public static SpeedController rightModuleMotor;
    
    public static DigitalInput angleZeroingButton;
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static final int leftMotor = 1;
    // public static final int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static final int rangefinderPort = 1;
    // public static final int rangefinderModule = 1;
    
    public static void init() {
                
        frontWheelEncoder = new Encoder(1, 1, 1, 2, false, CounterBase.EncodingType.k2X);
        frontWheelEncoder.setDistancePerPulse(WHEEL_ENCODER_DISTANCE_PER_TICK);
        frontWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        frontWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontWheelEncoder", frontWheelEncoder);
        leftWheelEncoder = new Encoder(1, 3, 1, 4, false, CounterBase.EncodingType.k2X);
        leftWheelEncoder.setDistancePerPulse(WHEEL_ENCODER_DISTANCE_PER_TICK); 
        leftWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        leftWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "leftWheelEncoder", leftWheelEncoder);
        backWheelEncoder = new Encoder(1, 5, 1, 6, false, CounterBase.EncodingType.k2X);
        backWheelEncoder.setDistancePerPulse(WHEEL_ENCODER_DISTANCE_PER_TICK); 
        backWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        backWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backWheelEncoder", backWheelEncoder); 
        rightWheelEncoder = new Encoder(1, 7, 1, 8, false, CounterBase.EncodingType.k2X);
        rightWheelEncoder.setDistancePerPulse(WHEEL_ENCODER_DISTANCE_PER_TICK); 
        rightWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        rightWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "rightWheelEncoder", rightWheelEncoder); 
        
        frontModuleEncoder = new Encoder(2, 1, 2, 2, false, CounterBase.EncodingType.k2X);
        frontModuleEncoder.setDistancePerPulse(MODULE_ENCODER_DISTANCE_PER_TICK); 
        frontModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        frontModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontModuleEncoder", frontModuleEncoder);
        leftModuleEncoder = new Encoder(2, 7, 2, 8, false, CounterBase.EncodingType.k2X);
        leftModuleEncoder.setDistancePerPulse(MODULE_ENCODER_DISTANCE_PER_TICK); 
        leftModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        leftModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "leftModuleEncoder", leftModuleEncoder);
        backModuleEncoder = new Encoder(2, 5, 2, 6, false, CounterBase.EncodingType.k2X);
        backModuleEncoder.setDistancePerPulse(MODULE_ENCODER_DISTANCE_PER_TICK); 
        backModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        backModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backModuleEncoder", backModuleEncoder);
        rightModuleEncoder = new Encoder(2, 3, 2, 4, false, CounterBase.EncodingType.k2X);
        rightModuleEncoder.setDistancePerPulse(MODULE_ENCODER_DISTANCE_PER_TICK); 
        rightModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        rightModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "rightModuleEncoder", rightModuleEncoder);
            
        frontWheelMotor = new Victor246(1,1);
        LiveWindow.addActuator("Drivetrain", "frontWheelMotor", (Victor) frontWheelMotor);
        leftWheelMotor = new Victor246(1,2);
        LiveWindow.addActuator("Drivetrain", "leftWheelMotor", (Victor) leftWheelMotor);
        backWheelMotor = new Victor246(1,3);
        LiveWindow.addActuator("Drivetrain", "backWheelMotor", (Victor) backWheelMotor);
        rightWheelMotor = new Victor246(1,4);
        LiveWindow.addActuator("Drivetrain", "rightWheelMotor", (Victor) rightWheelMotor);
        
        frontModuleMotor = new Jaguar246(2,1);
        LiveWindow.addActuator("Drivetrain", "frontModuleMotor", (Jaguar) frontModuleMotor);
        leftModuleMotor = new Jaguar246(2,2);
        LiveWindow.addActuator("Drivetrain", "leftModuleMotor", (Jaguar) leftModuleMotor);
        backModuleMotor = new Jaguar246(2,3);
        LiveWindow.addActuator("Drivetrain", "backModuleMotor", (Jaguar) backModuleMotor);
        rightModuleMotor = new Jaguar246(2,4);
        LiveWindow.addActuator("Drivetrain", "rightModuleMotor", (Jaguar) rightModuleMotor);
        
        angleZeroingButton = new DigitalInput(1,9);
        LiveWindow.addSensor("Drivetrain", "encoderZeroingSwitch", angleZeroingButton);
    }
}
