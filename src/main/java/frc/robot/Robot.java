package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;

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
    double straightLimit = 0.8;
    double twistLimit = 0.6;

    double twistDeadZone = 0.1;
    double straightDeadZone = 0.2;

    double baseSpeed = 0.4;

    // Auto Balance Parameters
    double levelAngle = 2.5;
    double maxCSDist = 5.0; 

    // Drive Motor Controllers
    PWMSparkMax rightBank;
    PWMSparkMax leftBank;
    DifferentialDrive drive;

    // Drive Motor Encoders
    RelativeEncoder rightBankEncoder;
    RelativeEncoder leftBankEncoder;

    // Collision Detection 
    double last_world_linear_accel_x = 0.0f;
    double last_world_linear_accel_y = 0.0f;
    boolean collisionDetected = false;
    boolean balanced = false;
    final static double kCollisionThreshold_DeltaG = 0.5f;
    Timer timer = new Timer();


    Encoder leftEncoder = new Encoder(0, 1);

    // Timings
    double timeNow;
    double timeStart;

    // IMU
    AHRS ahrs;

    double balanceSpeed = 0;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        leftEncoder.setDistancePerPulse(1. / 256.);

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

        collisionDetected = false;
        
        System.out.println("Initializing Teleoperated Driving");
        drive.tankDrive(0, 0);

        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                runDiagnostics();
            }
    
        }, 0, 50);
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
    private void runDiagnostics() {


        
        //initialize vars
        double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
               ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
              collisionDetected = true;
        }
        
        //SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);


    }

    private void runIMU() {
        
        
        double pitch = ahrs.getPitch();
        double distance = leftEncoder.getDistance();
        System.out.println("ENCODER DISTANCE:" + distance);
        SmartDashboard.putNumber( "encoderDistance", distance);
        SmartDashboard.putBoolean( "balanced", balanced);

        if (driveJoystick.getRawButtonPressed(2)) {
            leftEncoder.reset();
            balanced = false;
        }
        
        // System.out.println(ahrs.getVelocityX());
        if (driveJoystick.getTrigger()) {

            if (Math.abs(pitch) > levelAngle) {
                SmartDashboard.putBoolean( "angled", true);
                double speed = 0;
                if (!balanced && ((-0.5 < leftEncoder.getDistance() && leftEncoder.getDistance() < 0.6))) {
                    speed = (pitch / 90) + ((baseSpeed + 0.02) * Math.signum(pitch));
                }
                System.out.println(speed);
                balanced = false;
                drive.tankDrive(speed, speed);
                
            } else {
                SmartDashboard.putBoolean( "angled", false);
                balanced = true;
                leftEncoder.reset();
            }
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
            absRightBank -= absLeftBank - straightLimit;
            absLeftBank = straightLimit;
        }

        if (absRightBank > straightLimit) {
            absLeftBank -= absRightBank - straightLimit;
            absRightBank = straightLimit;
        }

        drive.tankDrive((absLeftBank * leftSign), (absRightBank * rightSign));
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

    private void encoderDrive(DifferentialDrive drive, double leftSpeed, double rightSpeed, double leftPulsesPerTick,
            double rightPulsesPerTick) {

        double encoderRatio = leftPulsesPerTick / rightPulsesPerTick;
        double expectedRatio = leftSpeed / rightSpeed;

        if (Math.abs(encoderRatio / expectedRatio) < 1) {
            rightSpeed *= Math.abs(encoderRatio / expectedRatio);
        } else {
            leftSpeed /= Math.abs(encoderRatio / expectedRatio);
        }

        drive.tankDrive(leftSpeed, rightSpeed);
    }
}
