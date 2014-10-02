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

    public AnalogChannel modulePot;

    public SpeedController wheelMotor;

    public SpeedController moduleMotor;
    
    PIDController speedPID;
    PIDController anglePID;
    
    public SwerveModule(Encoder encoder, AnalogChannel modulePot, SpeedController wheelMotor, SpeedController moduleMotor)
    {
        
    }
}
