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
    // supported platforms (Main robot and DrivePractice robot)

    // // Drive Motor Controllers
    PWMSparkMax testMotor;

    // Drive Motor Encoders
    Encoder testEncoder;


    System.out.println("Initializing Robot");
    
    testEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    }

    @Override
    public void teleopInit() {
        // timeStart = System.currentTimeMillis();
        
        // System.out.println("Initializing Teleoperated Driving");
        // drive.tankDrive(0, 0);
    }

    @Override
    public void teleopPeriodic() {
        // runDrive();
    }

    @Override
    public void testPeriodic() {
    }

    // Initializes main robot
    private void initMainRobot() {
        // leftBank = new PWMSparkMax(0);
        testMotor = new PWMSparkMax(0);
        testMotor.set(0.07);
        // rightBank.setInverted(true);
        
        // // leftBankEncoder = leftBank.getEncoder();
        // // rightBankEncoder = rightBank.getEncoder();

        // drive = new DifferentialDrive(leftBank, rightBank);
    }

    /**
     * Runs all of the Drive system operations.
     */
    private void runDrive() {

        // if (!enableDrive) {
        //     System.out.println("Drive Disabled");

        //     drive.tankDrive(0.0, 0.0);
        //     return;
        // }

        // // teleopDrive();
        // runIMU();
    }

    // private void runIMU() {
    //     double pitch = ahrs.getPitch();
    //     if (Math.abs(pitch) > 2.5 && driveJoystick.getRawButton(11)) {
    //         double speed = (pitch / 90) + (0.35 * Math.signum(pitch));
    //         drive.tankDrive(speed, speed);
            
    //         return;
    //     } else {
    //         teleopDrive(); 

    //         return;
    //     }
    // }

    // // Runs periodically when driving in teleoperated mode
    // private void teleopDrive() {
    //     double axisStraight = driveJoystick.getRawAxis(1);
    //     double axisTwist = driveJoystick.getRawAxis(2);
        
    //     double speedStraight = mapSpeed(axisStraight, baseSpeed, speedLimit, straightDeadZone);
    //     double speedTwist = mapSpeed(axisTwist, 0, twistLimit, twistDeadZone);

    //     double rawLeftBankSpeed = speedStraight - speedTwist;
    //     double rawRightBankSpeed = speedStraight + speedTwist;
        
    //     double leftSign = Math.signum(rawLeftBankSpeed);
    //     double absLeftBank = Math.abs(rawLeftBankSpeed);

    //     double rightSign = Math.signum(rawRightBankSpeed);
    //     double absRightBank = Math.abs(rawRightBankSpeed);

    //     if (absLeftBank > speedLimit) {
    //         absLeftBank = speedLimit;
    //         rightBankSpeed -= absLeftBank - speedLimit;
    //     }

    //     if (absRightBank > speedLimit) {
    //         absRightBank = speedLimit;
    //         leftBankSpeed -= absRightBank - speedLimit;
    //     }

    //     leftBankSpeed += ((absLeftBank * leftSign) - leftBankSpeed) / 2;
    //     rightBankSpeed += ((absRightBank * rightSign) - rightBankSpeed) / 2;

    //     drive.tankDrive(leftBankSpeed, rightBankSpeed);
    // }

    // // Resets encoder value and returns position
    // private double resetEncoderValue(RelativeEncoder encoder) {
    //     double value = encoder.getPosition();
    //     encoder.setPosition(0);
    //     return value;
    // }

    // private double mapSpeed(double speed, double min, double max, double deadZone) {        
    //     double sign = Math.signum(speed);
    //     double absSpeed = Math.abs(speed);

    //     if (absSpeed < deadZone) {
    //         return 0.0;
    //     } else {
    //         return mapNumber(absSpeed, deadZone, 1, min, max) * sign;
    //     }
    // }

    // private double mapNumber(double x, double a, double b, double c, double d) {
    //     return (x - a) / (b - a) * (d - c) + c;
    // }
}
