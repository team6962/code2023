package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.kauailabs.navx.frc.AHRS;
import java.util.TimerTask;
import java.util.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

    /*****************************************/
    /******** Configuration Variables ********/
    /*****************************************/


    // ENABLED SYSTEMS
    boolean enableDrive = true;
    boolean enableBalance = true;
    boolean enableArm = true;


    // DRIVER ADJUSTMENTS
    double straightLimit = 0.4; // Hard limit on power when going forward / backward
    double turnLimit = 0.4; // Hard limit on power when turning

    double twistDeadZone = 0.2; // Joystick deadzone for turning
    double straightDeadZone = 0.1; // Joystick deadzone for turning
    double throttleDeadZone = 0.1; // Joystick deadzone for arm lifting

    double basePower = 0; // Motor power required to get the chassis moving


    // AUTO-BALANCING
    double balanceLevelAngle = 2.5; // Angle range required to declare leveled
    double balanceAngleMultiple = 2; // Motor power multiple based on current angle
    double balanceBasePower = 0.25; // Base balancing speed


    // ARM EXTENSION
    double armExtendLimit = 33; // Arm extend limit (measured in encoder ticks)
    double armExtendPadding = 1; // Padding to prevent overshooting limits (measured in encoder ticks)
    double armExtendBasePower = 0.15; // Slowest speed arm will extend (0 - 1)
    double armExtendAddedPower = 0.2; // Additional speed added to base when far from limits (0 - 1)


    // ARM LIFTING
    double armLiftPower = 0.15; // Max arm lifting power
    double armLiftMinAngle = 20; // Min arm angle (degrees)
    double armLiftMaxAngle = 70; // Max arm angle (degrees)


    // DEVICE IDS
    int CAN_leftBank1 = 10;
    int CAN_leftBank2 = 28;
    int CAN_rightBank1 = 7;
    int CAN_rightBank2 = 27;
    int CAN_armLift1 = 5;
    int CAN_armLift2 = 15;
    int CAN_armExtend = 13;

    int DIO_armLiftEncoder = 0;

    int USB_driveJoystick = 0;
    int USB_utilityJoystick = 1;


    /****************************************/
    /*********** Global Variables ***********/
    /****************************************/

    // JOYSTICKS
    Joystick driveJoystick, utilityJoystick;

    // MOTOR CONTROLLERS
    MotorControllerGroup rightBank, leftBank, armLift;
    CANSparkMax leftBank1, leftBank2, rightBank1, rightBank2, armLift1, armLift2, armExtend;
    DifferentialDrive drive;

    // ENCODERS
    RelativeEncoder rightBankEncoder, leftBankEncoder, armExtendEncoder;
    DutyCycleEncoder armLiftEncoder;

    // TIMINGS
    double timeNow, timeStart;

    // IMU
    AHRS IMU;

    // Calibration
    boolean doingCalibration = false;
    double calibratingTicks = 0.0;


    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");


        /************** DRIVE SETUP *************/

        leftBank1 = new CANSparkMax(CAN_leftBank1, MotorType.kBrushless);
        leftBank2 = new CANSparkMax(CAN_leftBank2, MotorType.kBrushless);
        rightBank1 = new CANSparkMax(CAN_rightBank1, MotorType.kBrushless);
        rightBank2 = new CANSparkMax(CAN_rightBank2, MotorType.kBrushless);

        leftBank1.restoreFactoryDefaults();
        leftBank2.restoreFactoryDefaults();
        rightBank1.restoreFactoryDefaults();
        rightBank2.restoreFactoryDefaults();

        leftBank1.setIdleMode(IdleMode.kBrake);
        leftBank2.setIdleMode(IdleMode.kBrake);
        rightBank1.setIdleMode(IdleMode.kBrake);
        rightBank2.setIdleMode(IdleMode.kBrake);

        rightBank = new MotorControllerGroup(rightBank1, rightBank2);
        leftBank = new MotorControllerGroup(leftBank1, leftBank2);
        leftBank.setInverted(true);

        drive = new DifferentialDrive(leftBank, rightBank);

        leftBankEncoder = leftBank1.getEncoder();
        rightBankEncoder = rightBank1.getEncoder();


        /*************** ARM SETUP **************/

        armLift1 = new CANSparkMax(CAN_armLift1, MotorType.kBrushless);
        armLift2 = new CANSparkMax(CAN_armLift2, MotorType.kBrushless);
        armExtend = new CANSparkMax(CAN_armExtend, MotorType.kBrushless);

        armLift1.restoreFactoryDefaults();
        armLift2.restoreFactoryDefaults();
        armExtend.restoreFactoryDefaults();

        armLift1.setIdleMode(IdleMode.kBrake);
        armLift2.setIdleMode(IdleMode.kBrake);
        armExtend.setIdleMode(IdleMode.kBrake);

        armLift2.setInverted(true);
        armLift = new MotorControllerGroup(armLift1, armLift2);

        armExtend.setInverted(true);
        armExtend.setSoftLimit(SoftLimitDirection.kForward, (float) armExtendLimit);
        armExtend.setSoftLimit(SoftLimitDirection.kReverse, (float) 0);

        armExtendEncoder = armExtend.getEncoder();
        armLiftEncoder = new DutyCycleEncoder(DIO_armLiftEncoder);
        armLiftEncoder.setDistancePerRotation(360.0);


        /********** DRIVER STATION SETUP ********/

        driveJoystick = new Joystick(USB_driveJoystick);
        utilityJoystick = new Joystick(USB_utilityJoystick);


        /*************** IMU SETUP **************/

        IMU = new AHRS(I2C.Port.kMXP);
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
        if (enableDrive) {
            runDrive();
        } else {
            System.out.println("Drive Disabled");
            drive.tankDrive(0, 0);
        }

        if (enableBalance) {
            runBalance();
        } else {
            System.out.println("Balance Disabled");
        }

        if (enableArm) {
            runArm();
        } else {
            System.out.println("Arm Disabled");
        }
    }

    @Override
    public void testInit() {
        leftBank1.setIdleMode(IdleMode.kCoast);
        leftBank2.setIdleMode(IdleMode.kCoast);
        rightBank1.setIdleMode(IdleMode.kCoast);
        rightBank2.setIdleMode(IdleMode.kCoast);

        armLift1.setIdleMode(IdleMode.kCoast);
        armLift2.setIdleMode(IdleMode.kCoast);

        armExtend.setIdleMode(IdleMode.kCoast);
    }


    // Called periodically in test mode
    @Override
    public void testPeriodic() {
        if (doingCalibration) {
            calibrateBasePower();
        } else if (driveJoystick.getTrigger()) {
            calibrateBasePowerInit();
        }
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


    // Runs periodically when driving in teleoperated mode
    private void runDrive() {
        double straightAxis = driveJoystick.getRawAxis(1);
        double twistAxis = driveJoystick.getRawAxis(2);

        double straightPower = mapPower(straightAxis, basePower, straightLimit, straightDeadZone);
        double turningPower = mapPower(twistAxis, 0, turnLimit, twistDeadZone);

        drive.arcadeDrive(straightPower, turningPower);
    }


    public void runArm() {
        if (!driveJoystick.getTrigger()) {
            armExtend.set(0);
            armLift.set(0);
            return;
        }

        double extendPos = armExtendEncoder.getPosition();
        double liftAngle = armLiftEncoder.getDistance();
        double pov = driveJoystick.getPOV();

        if ((pov == 0 || pov == 315 || pov == 45) && extendPos <= armExtendLimit) {
            armExtend.set(mapNumber(extendPos, armExtendPadding, armExtendLimit - armExtendPadding, armExtendAddedPower, 0) + armExtendBasePower);
        } else if ((pov == 180 || pov == 225 || pov == 135) && extendPos >= 0) {
            armExtend.set(-(mapNumber(extendPos, armExtendPadding, armExtendLimit - armExtendPadding, 0, armExtendAddedPower) + armExtendBasePower));
        } else {
            armExtend.set(0);
        }

        if (liftAngle >= armLiftMinAngle && liftAngle <= armLiftMaxAngle) {
            armLift.set(-mapPower(driveJoystick.getRawAxis(3), 0, armLiftPower, throttleDeadZone));
        } else {
            armLift.set(0);
        }
    }


    private void runBalance() {
        if (!driveJoystick.getRawButton(11)) {
            drive.tankDrive(0, 0);
            return;
        }

        double pitch = IMU.getPitch();
        if (Math.abs(pitch) > balanceLevelAngle) {
            double power = (pitch / 360 * balanceAngleMultiple) + (balanceBasePower * Math.signum(pitch));
            drive.tankDrive(power, power);
        }
    }


    private double mapPower(double power, double min, double max, double deadZone) {
        double sign = Math.signum(power);
        double absPower = Math.abs(power);

        if (absPower < deadZone) {
            return 0.0;
        } else {
            return mapNumber(absPower, deadZone, 1, min, max) * sign;
        }
    }


    private double mapNumber(double x, double a, double b, double c, double d) {
        return (x - a) / (b - a) * (d - c) + c;
    }


    private void calibrateBasePowerInit() {
        System.out.println("Calibrating Speed...");
        doingCalibration = true;
        calibratingTicks = 0.0;
    }

    private void calibrateBasePower() {
        drive.tankDrive(calibratingTicks / 100, calibratingTicks / 100);
        System.out.println("Testing speed of " + String.valueOf(calibratingTicks / 100));

        if (leftBankEncoder.getVelocity() * rightBankEncoder.getVelocity() != 0) {

            basePower = calibratingTicks / 100;
            System.out.println("Done!");
            System.out.println("Set basePower to: " + String.valueOf(basePower));

            doingCalibration = false;
            calibratingTicks = 0.0;
            return;
        }

        calibratingTicks++;
    }
}
