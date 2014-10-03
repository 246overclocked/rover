package edu.wpi.first.wpilibj.templates;

import AwfulNameViaPaul.Potentiometer;
import edu.wpi.first.wpilibj.CounterBase;
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
    // Drivetrain constructors  
    // sensors
        // wheel speed 
    public static Encoder frontLeftWheelEncoder;
    public static Encoder frontRightWheelEncoder;
    public static Encoder backLeftWheelEncoder;
    public static Encoder backRightWheelEncoder;
        // wheel angle direction 
    public static Potentiometer frontLeftModulePot;
    public static Potentiometer frontRightModulePot;
    public static Potentiometer backLeftModulePot;
    public static Potentiometer backRightModulePot;
    
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
        frontLeftWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontLeftWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        frontLeftWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontLeftWheelEncoder", frontLeftWheelEncoder);
        frontRightWheelEncoder = new Encoder(3, 4, false);
        frontRightWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        frontRightWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        frontRightWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "frontRightWheelEncoder", frontRightWheelEncoder);
        backLeftWheelEncoder = new Encoder(5, 6, false);
        backLeftWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        backLeftWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        backLeftWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backLeftWheelEncoder", backLeftWheelEncoder); 
        backRightWheelEncoder = new Encoder(7, 8, false);
        backRightWheelEncoder.setDistancePerPulse(1); //TODO: set the distance travelled per pulse -- ned to test
        backRightWheelEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate); // have encoder measure rate, not distance
        backRightWheelEncoder.start();
        LiveWindow.addSensor("Drivetrain", "backRightWheelEncoder", backRightWheelEncoder); 
        
        frontLeftModulePot = new Potentiometer(9);
        frontLeftModulePot.setConversions(1, 2.5); // TODO set degreesPerVolt when mechanical team knows gear ratio
        LiveWindow.addSensor("Drivetrain", "frontLeftModulePot", frontLeftModulePot);
        frontRightModulePot = new Potentiometer(10);
        frontRightModulePot.setConversions(1, 2.5); // TODO set degreesPerVolt when mechanical team knows gear ratio
        LiveWindow.addSensor("Drivetrain", "frontRightModulePot", frontRightModulePot);
        backLeftModulePot = new Potentiometer(11);
        backLeftModulePot.setConversions(1, 2.5); // TODO set degreesPerVolt when mechanical team knows gear ratio
        LiveWindow.addSensor("Drivetrain", "backLeftModulePot", backLeftModulePot);
        backRightModulePot = new Potentiometer(12);
        backRightModulePot.setConversions(1, 2.5); // TODO set degreesPerVolt when mechanical team knows gear ratio
        LiveWindow.addSensor("Drivetrain", "backRightModulePot", backRightModulePot);
            
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
    }
}
