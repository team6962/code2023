package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {

    // Drive Enabled?
    private boolean enableDrive = true;

    // Joysticks
    Joystick driveJoystick;
    Joystick utilityJoystick;

    // Drive speed limits
    double straightLimit = 0.6;
    double twistLimit = 0.6;

    double twistDeadZone = 0.2;
    double straightDeadZone = 0.2;

    double baseSpeed = 0.33;

    // Auto Balance Parameters
    double levelAngle = 2.5;

    // Drive Motor Controllers
    PWMSparkMax rightBank;
    PWMSparkMax leftBank;
    DifferentialDrive drive;

    // Drive Motor Encoders
    RelativeEncoder rightBankEncoder;
    RelativeEncoder leftBankEncoder;

    // Timings
    double timeNow;
    double timeStart;

    // IMU
    AHRS ahrs;

    double leftBankSpeed = 0;
    double rightBankSpeed = 0;
    double balanceSpeed = 0;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");
        
        driveJoystick = new Joystick(0);
        utilityJoystick = new Joystick(1);
        
        try {
            ahrs = new AHRS(I2C.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX XMP: " + ex.getMessage(), true);
        }
        
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
        drive.tankDrive(0, 0);
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
        leftBank = new PWMSparkMax(0);
        rightBank = new PWMSparkMax(1);
        rightBank.setInverted(true);
        
        // leftBankEncoder = leftBank.getEncoder();
        // rightBankEncoder = rightBank.getEncoder();

        drive = new DifferentialDrive(leftBank, rightBank);
    }

    // Runs periodically when driving
    private void runDrive() {
        if (!enableDrive) {
            System.out.println("Drive Disabled");

            drive.tankDrive(0.0, 0.0);
            return;
        }

        // teleopDrive();
        runIMU();
    }

    private void runIMU() {
        double pitch = ahrs.getPitch();
        // System.out.println(ahrs.getVelocityX());
        if (driveJoystick.getTrigger() && Math.abs(pitch) > levelAngle) {
            double speed = (pitch / 90) + ((baseSpeed + 0.02) * Math.signum(pitch));
            // if (Math.abs(pitch) < levelAngle) {
            //     balanceSpeed = 0.0;
            // }

            // double velocity = ahrs.getVelocityX();
            
            // // If not moving when trying to...
            // if (Math.abs(velocity) < 0.1) {
            //     balanceSpeed += 0.02;
            // } else if (Math.abs(velocity) > 0.3) {
            //     balanceSpeed -= 0.02;
            // }

            // double speed = (balanceSpeed + baseSpeed) * Math.signum(pitch);

            // // double speed = (pitch / 90) + ((baseSpeed + 0.05) * Math.signum(pitch));
            
            System.out.println(speed);

            drive.tankDrive(speed, speed);
            return;
        } else {
            teleopDrive(); 
            return;
        }
    }

    // Runs periodically when driving in teleoperated mode
    private void teleopDrive() {
        double axisStraight = driveJoystick.getRawAxis(1);
        double axisTwist = driveJoystick.getRawAxis(2);
        
        double speedStraight = mapSpeed(axisStraight, baseSpeed, straightLimit, straightDeadZone);
        double speedTwist = mapSpeed(axisTwist, 0, twistLimit, twistDeadZone);

        double rawLeftBankSpeed = speedStraight - speedTwist;
        double rawRightBankSpeed = speedStraight + speedTwist;
        
        double leftSign = Math.signum(rawLeftBankSpeed);
        double absLeftBank = Math.abs(rawLeftBankSpeed);

        double rightSign = Math.signum(rawRightBankSpeed);
        double absRightBank = Math.abs(rawRightBankSpeed);

        if (absLeftBank > straightLimit) {
            rightBankSpeed -= absLeftBank - straightLimit;
            absLeftBank = straightLimit;
        }

        if (absRightBank > straightLimit) {
            leftBankSpeed -= absRightBank - straightLimit;
            absRightBank = straightLimit;
        }

        leftBankSpeed += ((absLeftBank * leftSign) - leftBankSpeed) / 5;
        rightBankSpeed += ((absRightBank * rightSign) - rightBankSpeed) / 5;

        drive.tankDrive(leftBankSpeed, rightBankSpeed);
    }

    // Resets encoder value and returns position
    private double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
    }

    private double mapSpeed(double speed, double min, double max, double deadZone) {        
        double sign = Math.signum(speed);
        double absSpeed = Math.abs(speed);

        if (absSpeed < deadZone) {
            return 0.0;
        } else {
            return mapNumber(absSpeed, deadZone, 1, min, max) * sign;
        }
    }

    private double mapNumber(double x, double a, double b, double c, double d) {
        return (x - a) / (b - a) * (d - c) + c;
    }
}
