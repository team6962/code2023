package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;

enum HM {
    NONE,
    UP,
    DOWN
}


public class Robot extends TimedRobot {

    Compressor c;
    DoubleSolenoid solenoid;
    Joystick joystick;


    @Override
    public void robotInit() {
        

        // Joystick
        joystick = new Joystick(0);

        c = new Compressor(0, PneumaticsModuleType.CTREPCM);
        c.enableDigital();
        // c.disable();

        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        solenoid.set(DoubleSolenoid.Value.kOff);
        
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {
       
        
        

    }

    @Override
    public void teleopInit() {
        
    }

    @Override
    public void teleopPeriodic() {
        System.out.println(solenoid.get());
        
        if (joystick.getRawButton(7)) {
            solenoid.set(DoubleSolenoid.Value.kForward);
        }
        if (joystick.getRawButton(8)) {
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Initializes the DrivePractice robot (as opposed to the Main robot)
     * 
     */
    

    
    
}
