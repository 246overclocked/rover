package edu.wpi.first.wpilibj.templates;

import Libraries.CANJaguar246;
import Libraries.Victor246;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
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
    public static final double WHEEL_TOP_ABSOLUTE_SPEED = 11;
    
    public static final double MAX_MODULE_ANGLE = 3*360;
    
    //the constants for optimizing which direction the swerveModules will turn
    public static final double K_MODULE_ANGLE_DELTA = 1;
    public static final double K_MODULE_ANGLE_TWIST = 0;
    public static final double K_MODULE_ANGLE_REVERSE = 0;
    
    public static final double DIAGNOSTICS_LOOP_PERIOD = 0.02; //in seconds
    
    public static final double WHEEL_ENCODER_DISTANCE_PER_TICK = .00628;
    
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
    public static CANJaguar246 frontModuleMotor;
    public static CANJaguar246 leftModuleMotor;
    public static CANJaguar246 backModuleMotor;
    public static CANJaguar246 rightModuleMotor;
    
    public static DigitalInput angleZeroingButton;
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static final int leftMotor = 1;
    // public static final int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static final int rangefinderPort = 1;
    // public static final int rangefinderModule = 1;
    
    public static void init() throws CANTimeoutException{
                
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
            
        frontWheelMotor = new Victor246(1,1);
        LiveWindow.addActuator("Drivetrain", "frontWheelMotor", (Victor) frontWheelMotor);
        leftWheelMotor = new Victor246(1,2);
        LiveWindow.addActuator("Drivetrain", "leftWheelMotor", (Victor) leftWheelMotor);
        backWheelMotor = new Victor246(1,3);
        LiveWindow.addActuator("Drivetrain", "backWheelMotor", (Victor) backWheelMotor);
        rightWheelMotor = new Victor246(1,4);
        LiveWindow.addActuator("Drivetrain", "rightWheelMotor", (Victor) rightWheelMotor);
        
        frontModuleMotor = new CANJaguar246(0, CANJaguar.ControlMode.kPosition);
        frontModuleMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
        frontModuleMotor.setPID(.015, .0012, .003);
        frontModuleMotor.configEncoderCodesPerRev(250);
        frontModuleMotor.configSoftPositionLimits(MAX_MODULE_ANGLE, -MAX_MODULE_ANGLE);
        frontModuleMotor.enableControl(0);
        LiveWindow.addActuator("Drivetrain", "frontModuleMotor", frontModuleMotor);
        leftModuleMotor = new CANJaguar246(0, CANJaguar.ControlMode.kPosition);
        leftModuleMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
        leftModuleMotor.setPID(.015, .0012, .003);
        leftModuleMotor.configEncoderCodesPerRev(250);
        leftModuleMotor.configSoftPositionLimits(MAX_MODULE_ANGLE, -MAX_MODULE_ANGLE);
        leftModuleMotor.enableControl(0);
        LiveWindow.addActuator("Drivetrain", "leftModuleMotor", leftModuleMotor);
        backModuleMotor = new CANJaguar246(0, CANJaguar.ControlMode.kPosition);
        backModuleMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
        backModuleMotor.setPID(.015, .0012, .003);
        backModuleMotor.configEncoderCodesPerRev(250);
        backModuleMotor.configSoftPositionLimits(MAX_MODULE_ANGLE, -MAX_MODULE_ANGLE);
        backModuleMotor.enableControl(0);
        LiveWindow.addActuator("Drivetrain", "backModuleMotor", backModuleMotor);
        rightModuleMotor = new CANJaguar246(0, CANJaguar.ControlMode.kPosition);
        rightModuleMotor.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
        rightModuleMotor.setPID(.015, .0012, .003);
        rightModuleMotor.configEncoderCodesPerRev(250);
        rightModuleMotor.configSoftPositionLimits(MAX_MODULE_ANGLE, -MAX_MODULE_ANGLE);
        rightModuleMotor.enableControl(0);
        LiveWindow.addActuator("Drivetrain", "rightModuleMotor", rightModuleMotor);
        
        angleZeroingButton = new DigitalInput(1,9);
        LiveWindow.addSensor("Drivetrain", "encoderZeroingSwitch", angleZeroingButton);
    }
}
