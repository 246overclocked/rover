package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
    public static final double LEFT_RIGHT_WIDTH = 12; // 12in = 1ft
//    distance between "front" and "back" modules
    public static final double FRONT_BACK_LENGTH = 12;
//    it is assumed that the robot's center is located smack halfway between the
//    "front" and "back" modules and the "left" and "right" modules
    public static final double WHEEL_TOP_ABSOLUTE_SPEED = 20;
    
    //the constants for optimizing which direction the swerveModules will turn
    public static final double K_MODULE_ANGLE_DELTA = 1;
    public static final double K_MODULE_ANGLE_TWIST = 0;
    public static final double K_MODULE_ANGLE_REVERSE = 0;
    
    public static final double MAX_MODULE_ANGLE = 3*360;
    
    public static final double DIAGNOSTICS_LOOP_PERIOD = 0.02; //in seconds
    
// Drivetrain constructors  
    // sensors
        // wheel speed 
    public static Encoder frontLeftWheelEncoder;
    public static Encoder frontRightWheelEncoder;
    public static Encoder backLeftWheelEncoder;
    public static Encoder backRightWheelEncoder;
        // wheel angle direction 
    public static Encoder frontLeftModuleEncoder;
    public static Encoder frontRightModuleEncoder;
    public static Encoder backLeftModuleEncoder;
    public static Encoder backRightModuleEncoder;
    
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
        frontLeftWheelEncoder = new Encoder(1, 1, 1, 2, false);
        frontLeftWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        frontLeftWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontLeftWheelEncoder", frontLeftWheelEncoder);
        frontRightWheelEncoder = new Encoder(1, 3, 1, 4, false);
        frontRightWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontRightWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        frontRightWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontRightWheelEncoder", frontRightWheelEncoder);
        backLeftWheelEncoder = new Encoder(1, 5, 1, 6, false);
        backLeftWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        backLeftWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        backLeftWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backLeftWheelEncoder", backLeftWheelEncoder); 
        backRightWheelEncoder = new Encoder(1, 7, 1, 8, false);
        backRightWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        backRightWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        backRightWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backRightWheelEncoder", backRightWheelEncoder); 
        
        frontLeftModuleEncoder = new Encoder(1, 9, 1, 10, false);
        frontLeftModuleEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        frontLeftModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontLeftModuleEncoder", frontLeftModuleEncoder);
        frontRightModuleEncoder = new Encoder(2, 1, 2, 2, false);
        frontLeftModuleEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        frontLeftModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontRightModuleEncoder", frontRightModuleEncoder);
        backLeftModuleEncoder = new Encoder(2, 3, 2, 4, false);
        frontLeftModuleEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        frontLeftModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backLeftModuleEncoder", backLeftModuleEncoder);
        backRightModuleEncoder = new Encoder(2, 5, 2, 6, false);
        frontLeftModuleEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftModuleEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance); // have encoder measure rate, not distance
        frontLeftModuleEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backRightModuleEncoder", backRightModuleEncoder);
            
        frontLeftWheelMotor = new Victor(1);
        LiveWindow.addActuator("Drivetrain", "frontLeftWheelMotor", (Victor) frontLeftWheelMotor);
        frontRightWheelMotor = new Victor(2);
        LiveWindow.addActuator("Drivetrain", "frontRightWheelMotor", (Victor) frontRightWheelMotor);
        backLeftWheelMotor = new Victor(3);
        LiveWindow.addActuator("Drivetrain", "backLeftWheelMotor", (Victor) backLeftWheelMotor);
        backRightWheelMotor = new Victor(4);
        LiveWindow.addActuator("Drivetrain", "backRightWheelMotor", (Victor) backRightWheelMotor);
        
        frontLeftModuleMotor = new Victor(5);
        LiveWindow.addActuator("Drivetrain", "frontLeftModuleMotor", (Victor) frontLeftModuleMotor);
        frontRightModuleMotor = new Victor(6);
        LiveWindow.addActuator("Drivetrain", "frontRightModuleMotor", (Victor) frontRightModuleMotor);
        backLeftModuleMotor = new Victor(7);
        LiveWindow.addActuator("Drivetrain", "backLeftModuleMotor", (Victor) backLeftModuleMotor);
        backRightModuleMotor = new Victor(8);
        LiveWindow.addActuator("Drivetrain", "backRightModuleMotor", (Victor) backRightModuleMotor);
        
        angleZeroingButton = new DigitalInput(1);
        LiveWindow.addSensor("Drivetrain", "encoderZeroingSwitch", angleZeroingButton);
    }
}
