package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.kauailabs.navx.frc.AHRS;
import java.util.TimerTask;
import java.nio.file.ClosedWatchServiceException;
import java.util.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public class Robot extends TimedRobot {

    /*****************************************/
    /******** Configuration Variables ********/
    /*****************************************/


    // ENABLED SYSTEMS
    boolean enableDrive = false;
    boolean enableBalance = false;
    boolean enableArm = true;
    boolean enableClaw = false;
    boolean enablePneumaticClaw = false;
    boolean enableVision = false;


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


    // ARM POSITIONING
    double armExtendInches = 60; // Inches from pivot when fully extended
    double armRetractInches = 40; // Inches from pivot when fully retracted
    double armHeightInches = 40; // Inches above ground from pivot
    double armLiftEncoderOffset = 242; // Offset so encoder reads 90 degrees when parallel to ground


    // ARM EXTENSION
    double armExtendLimit = 33; // Arm extend limit (measured in encoder ticks)
    double armExtendPadding = 0.2; // Padding to prevent overshooting limits (measured in percent 0 - 1)
    double armExtendMinPower = 0.1; // Slowest speed arm will extend (0 - 1)
    double armExtendMaxPower = 0.3; // Fastest speed arm will extend (0 - 1)


    // ARM LIFTING
    double armLiftMaxPower = 0.15; // Max arm lifting power
    double armLiftPowerIncrement = 0.005; // Arm lifting power increment each tick
    double armLiftMinAngle = 28; // Min arm angle (degrees)
    double armLiftMaxAngle = 118; // Max arm angle (degrees)
    double armLiftAnglePrecision = 1; // Degrees of precision


    // CLAW
    double clawGrabLimit = -200; // Claw grab limit (measured in encoder ticks)
    double clawGrabPadding = 0.2; // Padding to prevent overshooting limits (measured in percent 0 - 1)
    double clawGrabMinPower = 0.05; // Slowest speed claw will grab (0 - 1)
    double clawGrabMaxPower = 0.4; // Fastest speed claw will grab (0 - 1)


    // DEVICE IDS
    int CAN_leftBank1 = 10;
    int CAN_leftBank2 = 28;
    int CAN_rightBank1 = 7;
    int CAN_rightBank2 = 27;
    int CAN_armLift1 = 5;
    int CAN_armLift2 = 15;
    int CAN_armExtend = 13;
    int CAN_clawGrab = 16;

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
    CANSparkMax leftBank1, leftBank2, rightBank1, rightBank2, armLift1, armLift2, armExtend, clawGrab;
    DifferentialDrive drive;

    // ENCODERS
    RelativeEncoder rightBankEncoder, leftBankEncoder, armExtendEncoder, clawGrabEncoder;
    DutyCycleEncoder armLiftEncoder;

    // DIGITAL INPUTS
    DigitalInput clawStop;

    // TIMINGS
    double timeNow, timeStart;

    // IMU
    AHRS IMU;

    // CALIBRATION
    boolean doingCalibration = false;
    double calibratingTicks = 0.0;

    // PNEUMATICS
    Compressor clawCompressor;
    DoubleSolenoid clawSolenoid;

    // ARM LIFTING
    double armLiftBasePower = 0;
    double armLiftTargetAngle = 0;
    double p_armLiftEncoder = 0;


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
        armLiftEncoder.setPositionOffset(armLiftEncoderOffset / 360.0);
        armLiftEncoder.setDistancePerRotation(360.0);

        p_armLiftEncoder = armLiftEncoder.getDistance();
        armLiftTargetAngle = armLiftEncoder.getDistance();


        /************** CLAW SETUP **************/

        if (enableClaw) {

            clawGrab = new CANSparkMax(CAN_clawGrab, MotorType.kBrushless);

            clawGrab.restoreFactoryDefaults();

            clawGrab.setIdleMode(IdleMode.kBrake);

            clawGrab.setSoftLimit(SoftLimitDirection.kForward, (float) clawGrabLimit);
            clawGrab.setSoftLimit(SoftLimitDirection.kReverse, (float) 0);

            clawGrabEncoder = clawGrab.getEncoder();

            clawStop = new DigitalInput(1);
        }

        /********** PNEUMATIC CLAW SETUP ********/

        clawCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        clawCompressor.enableDigital();

        if (!enablePneumaticClaw) {
            clawCompressor.disable();
        }

        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        clawSolenoid.set(DoubleSolenoid.Value.kOff);

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

        leftBank1.setIdleMode(IdleMode.kBrake);
        leftBank2.setIdleMode(IdleMode.kBrake);
        rightBank1.setIdleMode(IdleMode.kBrake);
        rightBank2.setIdleMode(IdleMode.kBrake);

        armLift1.setIdleMode(IdleMode.kBrake);
        armLift2.setIdleMode(IdleMode.kBrake);

        armExtend.setIdleMode(IdleMode.kBrake);

        if (enableClaw) {
            clawGrab.setIdleMode(IdleMode.kBrake);
        }

        System.out.println("Initializing Teleoperated Driving");
        drive.tankDrive(0, 0);
    }


    // Called periodically in teleoperated mode
    @Override
    public void teleopPeriodic() {
        if (enableDrive) {
            runDrive();
        } else {
            drive.tankDrive(0, 0);
        }

        if (enableBalance && driveJoystick.getRawButton(11)) {
            runBalance();
        }

        if (enableArm) {
            runArm();
        }

        if (enableClaw) {
            runClaw();
        }

        if (enablePneumaticClaw) {
            runPneumaticClaw();
        }

        if (enableVision) {
            runVision();
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

        if (enableClaw) {
            clawGrab.setIdleMode(IdleMode.kCoast);
        }
    }


    // Called periodically in test mode
    @Override
    public void testPeriodic() {
        if (doingCalibration) {
            calibrateBasePower();
        } else if (driveJoystick.getTrigger()) {
            calibrateBasePowerInit();
        }
        System.out.println(armLiftEncoder.getDistance());
    }


    // Called periodically when robot is enabled
    @Override
    public void robotPeriodic() {
        updateDashboard();
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


    private void runArm() {
        
        // Arm Extension

        double extendPos = armExtendEncoder.getPosition();
        double pov = driveJoystick.getPOV();

        double maxArmExtension = Math.min(armExtendLimit, ((armHeightInches / Math.cos(armLiftEncoder.getDistance() / 180 * Math.PI) - armRetractInches) / (armExtendInches - armRetractInches) * armExtendLimit));
        
        if (armLiftEncoder.getDistance() > 90) {
            maxArmExtension = armExtendLimit;
        }

        int extendDirection = 0;
        double extendPower = armExtendMaxPower;

        if (driveJoystick.getRawButton(6) || driveJoystick.getRawButton(4)) {
            extendPower = armExtendMinPower;
        }

        if (pov == 0 || pov == 315 || pov == 45 || driveJoystick.getRawButton(6)) {
            extendDirection = 1;
        } else if (pov == 180 || pov == 225 || pov == 135 || driveJoystick.getRawButton(4)) {
            extendDirection = -1;
        }

        // System.out.println(armHeightInches / Math.cos(armLiftEncoder.getDistance() / 180 * Math.PI));
        // System.out.println(maxArmExtension);
        System.out.println(maxArmExtension - extendPos);
        armExtend.set(mapLimitedPower(extendDirection, extendPos, 0, maxArmExtension, armExtendMinPower, extendPower, armExtendPadding * maxArmExtension));


        // Arm Lifting

        double liftAngle = armLiftEncoder.getDistance();
        double rangeOfMotion = armLiftMaxAngle - armLiftMinAngle;

        if (driveJoystick.getTrigger()) {
            armLiftTargetAngle = mapNumber(driveJoystick.getRawAxis(3), -1, 1, armLiftMaxAngle, armLiftMinAngle);
        }

        armLiftTargetAngle = Math.max(armLiftTargetAngle, armLiftMinAngle);
        armLiftTargetAngle = Math.min(armLiftTargetAngle, armLiftMaxAngle);

        if (Math.abs(armLiftTargetAngle - liftAngle) > armLiftAnglePrecision) {
            double increment = mapNumber(armLiftTargetAngle - liftAngle, -rangeOfMotion / 5, rangeOfMotion / 5, -armLiftMaxPower, armLiftMaxPower) / 20;

            if (
                (armLiftTargetAngle > liftAngle && liftAngle - p_armLiftEncoder < 0.05) || // Need to go up
                (armLiftTargetAngle < liftAngle && liftAngle - p_armLiftEncoder > -0.05)    // Need to go down
            ) {
                armLiftBasePower += increment;
            }

            // System.out.println(armLiftTargetAngle - liftAngle);
        }

        double armLiftPower = armLiftBasePower + mapNumber(armLiftTargetAngle - liftAngle, -rangeOfMotion / 2, rangeOfMotion / 2, -armLiftMaxPower, armLiftMaxPower);

        if (liftAngle > armLiftMaxAngle) {
            armLiftPower = Math.min(0, armLiftPower);
        }

        if (liftAngle < armLiftMinAngle || (extendPos > maxArmExtension && maxArmExtension != armExtendLimit)) {
            armLiftPower = Math.max(0, armLiftPower);
        }

        armLiftPower = Math.min(armLiftMaxPower, Math.abs(armLiftPower)) * Math.signum(armLiftPower);

        armLift.set(armLiftPower);

        p_armLiftEncoder = liftAngle;
    }


    private void runBalance() {
        double pitch = IMU.getPitch();
        if (Math.abs(pitch) > balanceLevelAngle) {
            double power = -((pitch / 360 * balanceAngleMultiple) + (balanceBasePower * Math.signum(pitch)));
            drive.tankDrive(power, power);
        }
    }


    private void runClaw() {
        double grabPos = clawGrabEncoder.getPosition();
        System.out.println(clawStop.get());
        if (driveJoystick.getRawButton(5)) { // Closing
            if (!clawStop.get()) {
                clawGrab.set(0);
                return;
            }
            clawGrab.set(mapLimitedPower(1, grabPos, clawGrabLimit, 0, clawGrabMinPower, clawGrabMaxPower, clawGrabPadding * clawGrabLimit));
        } else if (driveJoystick.getRawButton(3)) { // Opening
            clawGrab.set(mapLimitedPower(-1, grabPos, clawGrabLimit, 0, clawGrabMinPower, clawGrabMaxPower, clawGrabPadding * clawGrabLimit));
        } else {
            clawGrab.set(0);
        }
    }

    private void runPneumaticClaw() {
        if (driveJoystick.getRawButton(5)) {
            clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (driveJoystick.getRawButton(3)) {
            clawSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    private void runVision() {

    }


    /* 
    TODO: Prevent arm from extending into the ground
    
    TODO: Add smartdashboard stuff
        Things to add:
        - Claw state (open / closed / at limits)
        - Solenoid state (opening or closing)
        - Arm state (Angle, extend position, if in range of preset)
        - Mode (teleop vs auton)
        - Camera feed
        - 3D position data
        - Drive state (Max power, left and right encoder speeds vs their motor power)
        - Balance state (if ready to balance, angle currently)
        - Enabled Systems
        - Coast vs Brake on all motor controllers (Group by system)
        - 
    */

    private void updateDashboard() {
        // ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");

        // dashboard.add("Drive", drive);
        // dashboard.add("Arm Angle", armLiftEncoder.getDistance()).withWidget("Gyro");
        // dashboard.add("Arm Extension %", (armExtendEncoder.getPosition() / armExtendLimit) * 100);
        // dashboard.add("Balance Angle", IMU.getPitch());
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

    private double mapLimitedPower(int direction, double pos, double minPos, double maxPos, double minPower, double maxPower, double padding) {
        if (direction > 0) {
            if (pos > maxPos) {
                return 0.0;
            }
            if (pos > maxPos - padding) {
                return mapNumber(pos, maxPos - padding, maxPos, maxPower, minPower);
            } else {
                return maxPower;
            }
        } else if (direction < 0) {
            if (pos < minPos) {
                return 0.0;
            }
            if (pos < minPos + padding) {
                return -mapNumber(pos, minPos, minPos + padding, minPower, maxPower);
            } else {
                return -maxPower;
            }
        } else {
            return 0.0;
        }
    }


    private double mapNumber(double x, double a, double b, double c, double d) {
        if (x < a) {
            return c;
        }
        if (x > b) {
            return d;
        }
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
