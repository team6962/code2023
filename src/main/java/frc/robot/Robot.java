package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {

    // // Drive Motor Controllers
    PWMSparkMax testMotor;
    double testMotor1Speed=.2;
    // Drive Motor Encoders
    Encoder testEncoder;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");
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
        testMotor.set(testMotor1Speed);
        double dist = testEncoder.getDistance();
        System.out.println(dist);
    }

    // Called periodically in test mode
    @Override
    public void testPeriodic() {
    }
}
