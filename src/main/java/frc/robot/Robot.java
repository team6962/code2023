package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
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

import com.kauailabs.navx.frc.AHRS;
import java.util.TimerTask;
import java.util.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    // Drive Enabled?
    private boolean enableDrive = true;

    // Joysticks
    Joystick driveJoystick;
    Joystick utilityJoystick;

    // Drive speed limits
    double straightLimit = 0.4;
    double twistLimit = 0.4;

    double twistDeadZone = 0.2;
    double straightDeadZone = 0.1;

    double baseSpeed = 0;

    // Auto Balance Parameters
    double levelAngle = 2.5;
    double maxCSDist = 5.0;

    // Drive Motor Controllers
    MotorControllerGroup rightBank;
    MotorControllerGroup leftBank;

    DifferentialDrive drive;

    // Drive Motor Encoders
    RelativeEncoder rightBankEncoder;
    RelativeEncoder leftBankEncoder;

    // Timings
    double timeNow;
    double timeStart;

    // IMU
    AHRS ahrs;

    // Calibration
    boolean calibratingSpeed = false;
    double calibratingTicks = 0.0;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");

        CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
        CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
        CANSparkMax right1 = new CANSparkMax(3, MotorType.kBrushless);
        CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);

        left1.setIdleMode(IdleMode.kBrake);
        left2.setIdleMode(IdleMode.kBrake);
        right1.setIdleMode(IdleMode.kBrake);
        right2.setIdleMode(IdleMode.kBrake);

        leftBank = new MotorControllerGroup(left1, left2);
        rightBank = new MotorControllerGroup(right1, right2);
        leftBank.setInverted(true);

        leftBankEncoder = left1.getEncoder();
        rightBankEncoder = right1.getEncoder();

        drive = new DifferentialDrive(leftBank, rightBank);

        driveJoystick = new Joystick(0);
        utilityJoystick = new Joystick(1);

        try {
            ahrs = new AHRS(I2C.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX XMP: " + ex.getMessage(), true);
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
        if (!enableDrive) {
            System.out.println("Drive Disabled");

            drive.tankDrive(0.0, 0.0);
            return;
        }

        if (driveJoystick.getTrigger()) {
            balance();
        } else {
            teleopDrive();
        }
    }

    // Called periodically in test mode
    @Override
    public void testPeriodic() {
        if (calibratingSpeed) {
            calibrateSpeed();
        } else if (driveJoystick.getTrigger()) {
            calibrateSpeedInit();
        }
    }

    private void balance() {
        double pitch = ahrs.getPitch();
        if (Math.abs(pitch) > levelAngle) {
            double speed = (pitch / 180) + ((baseSpeed + 0.25) * Math.signum(pitch));
            drive.tankDrive(speed, speed);
        }
    }

    // Runs periodically when driving in teleoperated mode
    private void teleopDrive() {
        double axisStraight = driveJoystick.getRawAxis(1);
        double axisTwist = driveJoystick.getRawAxis(2);

        double speedStraight = mapSpeed(axisStraight, baseSpeed, straightLimit, straightDeadZone);
        double speedTwist = mapSpeed(axisTwist, 0, twistLimit, twistDeadZone);

        drive.arcadeDrive(speedStraight, speedTwist);
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

    private void runDiagnostics() {

        // initialize vars
        double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) ||
                (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
            collisionDetected = true;
        }

        // SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);

    }

    private void calibrateSpeedInit() {
        System.out.println("Calibrating Speed...");
        calibratingSpeed = true;
        calibratingTicks = 0.0;
    }

    private void calibrateSpeed() {
        drive.tankDrive(calibratingTicks / 100, calibratingTicks / 100);
        System.out.println("Testing speed of " + String.valueOf(calibratingTicks / 100));

        if (leftBankEncoder.getVelocity() * rightBankEncoder.getVelocity() != 0) {

            baseSpeed = calibratingTicks / 100;
            System.out.println("Done!");
            System.out.println("Set baseSpeed to: " + String.valueOf(baseSpeed));

            calibratingSpeed = false;
            calibratingTicks = 0.0;
            return;
        }

        calibratingTicks++;
    }
}
