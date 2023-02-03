package frc.robot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {

    // // Drive Motor Controllers
    PWMSparkMax testMotor;
    double testMotor1Speed=.2;
    //int encoderHalfDistance = 52442;
    int encoderHalfDistance = 45000;
    Joystick joystick;
    boolean turnStatrted = false;
    double startDist = 0.0;

    // Drive Motor Encoders
    Encoder testEncoder;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");
        joystick = new Joystick(0);
        testEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    }

    // Called periodically when robot is enabled
    @Override
    public void robotPeriodic() {

    }

    @Override
    public void disabledPeriodic() {
    }

    // Called when autonomous mode is enabled
    @Override
    public void autonomousInit() {
    }

    // Called periodically in autonomous mode
    @Override
    public void autonomousPeriodic() {
    }

    // Called when teleoperated mode is enabled
    @Override
    public void teleopInit() {
        testMotor = new PWMSparkMax(1);
    }

    // Called periodically in teleoperated mode
    @Override
    public void teleopPeriodic() {

        double dist = testEncoder.getDistance();

        if (joystick.getRawButtonPressed(1) && !turnStatrted){
            turnStatrted = true;
            startDist = dist;
        } 
        
        if (turnStatrted){
            if (dist >= (encoderHalfDistance + startDist)) {
                testMotor.set(0);
                turnStatrted = false;
            } else {
                testMotor.set(testMotor1Speed);    
            }
            System.out.println(dist);
    
        }
        
        
    }

    // Called periodically in test mode
    @Override
    public void testPeriodic() {
    }
}
