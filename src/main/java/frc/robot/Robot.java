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

    // driving systems
    private boolean enableDrive = true;

    // Joystick
    Joystick driveJoystick;
    Joystick utilityJoystick1;

    // Drive speed limits
    double speedLimit = 0.8; 
    double twistLimit = 0.8;

    // Drive Motor Controllers
    MotorControllerGroup rightBank;
    MotorControllerGroup leftBank;
    DifferentialDrive myDrive;

    // Drive Motor Encoders
    RelativeEncoder rightBankEncoder;
    RelativeEncoder leftBankEncoder;

    // Drive Practice Controllers
    private PWMSparkMax drivePracticeLeftController;
    private PWMSparkMax drivePracticeRightController;

    double timeNow;
    double timeStart;

    boolean running = false;

    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");
        
        initMainRobot();

        driveJoystick = new Joystick(0);
        utilityJoystick1 = new Joystick(1);
    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void autonomousInit() {
        timeStart = System.currentTimeMillis();
        leftBankEncoder.setPosition(0);
        rightBankEncoder.setPosition(0);
    }

    @Override
    public void autonomousPeriodic() {
        timeNow = System.currentTimeMillis() - timeStart;
    }

    @Override
    public void teleopInit() {
        timeStart = System.currentTimeMillis();
        
        System.out.println("Initializing Teleoperated Driving")
        myDrive.tankDrive(0, 0);
    }

    @Override
    public void teleopPeriodic() {
        runDrive();
    }

    @Override
    public void testPeriodic() {

    }

    private double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
    }

    /**
     * Handles teleoperated driving mode.
     * 
     * @todo Currently janky, needs testing and fine-tuning.
     */
    private void teleopDriver() {
        double rawAxisForwardBack = -driveJoystick.getRawAxis(1);
        double rawAxisTwist = driveJoystick.getRawAxis(2);

        double limitAxisForwardBack = rawAxisSpeed * speedLimit
        double limitAxisTwist = rawAxisTwist * twistLimit
        
        double leftBankSpeed = limitAxisForwardBack + limitAxisTwist;
        double rightBankSpeed = limitAxisForwardBack - limitAxisTwist;
        
        myDrive.tankDrive(leftBankSpeed, -rightBankSpeed);
    }


    /**
     * Runs all of the Drive system operations.
     */
    private void runDrive() {
        if (!enableDrive) {
            System.out.println("Drive Disabled")

            myDrive.tankDrive(0.0, 0.0);
            return;
        }

        teleopDriver();
    }

    /**
     * Initializes the Main robot (as opposed to the DrivePractice robot)
     * 
     */
    private void initMainRobot() {
        // Drive motors
        rightBank = new MotorControllerGroup(
            new CANSparkMax(9, MotorType.kBrushless)
        );

        leftBank = new MotorControllerGroup(
            new CANSparkMax(4, MotorType.kBrushless)
        );
        
        leftBankEncoder = leftBankEncoderHold.getEncoder();
        rightBankEncoder = rightBankEncoderHold.getEncoder();

        myDrive = new DifferentialDrive(leftBank, rightBank);
    }

    /**
     * Initializes the DrivePractice robot (as opposed to the Main robot)
     * 
     */
    private void initDrivePracticeRobot() {
        drivePracticeRightController = new PWMSparkMax(0);
        drivePracticeLeftController = new PWMSparkMax(1);
        drivePracticeLeftController.setInverted(true);

        myDrive = new DifferentialDrive(drivePracticeLeftController, drivePracticeRightController);
    }
}
