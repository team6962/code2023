package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {

    // Drive Enabled?
    private boolean enableDrive = true;

    // Joysticks
    Joystick driveJoystick;
    Joystick utilityJoystick;

    // Drive speed limits
    double speedLimit = 0.8;
    double twistLimit = 0.8;

    // Drive Motor Controllers
    PWMSparkMax rightBank;
    PWMSparkMax leftBank;
    DifferentialDrive myDrive;

    // Drive Motor Encoders
    RelativeEncoder rightBankEncoder;
    RelativeEncoder leftBankEncoder;

    // Timings
    double timeNow;
    double timeStart;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");
        
        driveJoystick = new Joystick(0);
        utilityJoystick = new Joystick(1);

        initMainRobot();
    }

    // Called periodically when robot is enabled
    @Override
    public void robotPeriodic() {

    }

    // Called when autonomous mode is enabled
    @Override
    public void autonomousInit() {
        timeStart = System.currentTimeMillis();
    }

    // Called periodically in autonomous mode
    @Override
    public void autonomousPeriodic() {
        timeNow = System.currentTimeMillis() - timeStart;
    }

    // Called when teleoperated mode is enabled
    @Override
    public void teleopInit() {
        timeStart = System.currentTimeMillis();
        
        System.out.println("Initializing Teleoperated Driving");
        myDrive.tankDrive(0, 0);
    }

    // Called periodically in teleoperated mode
    @Override
    public void teleopPeriodic() {
        runDrive();
    }

    // Called periodically in test mode
    @Override
    public void testPeriodic() {

    }

    // Initializes main robot
    private void initMainRobot() {
        leftBank = new PWMSparkMax(1);
        rightBank = new PWMSparkMax(2);
        
        // leftBankEncoder = leftBank.getEncoder();
        // rightBankEncoder = rightBank.getEncoder();

        myDrive = new DifferentialDrive(leftBank, rightBank);
    }

    // Runs periodically when driving
    private void runDrive() {
        if (!enableDrive) {
            System.out.println("Drive Disabled");

            myDrive.tankDrive(0.0, 0.0);
            return;
        }

        teleopDrive();
    }

    // Runs periodically when driving in teleoperated mode
    private void teleopDrive() {
        double rawAxisForwardBack = -driveJoystick.getRawAxis(1);
        double rawAxisTwist = driveJoystick.getRawAxis(2);

        double limitAxisForwardBack = rawAxisForwardBack * speedLimit;
        double limitAxisTwist = rawAxisTwist * twistLimit;
        
        double leftBankSpeed = limitAxisForwardBack + limitAxisTwist;
        double rightBankSpeed = limitAxisForwardBack - limitAxisTwist;
        
        myDrive.tankDrive(leftBankSpeed, -rightBankSpeed);
    }

    // Resets encoder value and returns position
    private double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
    }
}
