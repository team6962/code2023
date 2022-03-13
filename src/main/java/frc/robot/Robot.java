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
    private int platformMain = 1;
    private int platformDrivePractice = 2;

    // current platform for build
    private int platformCurrent = platformDrivePractice;

    // driving systems
    private boolean enableDrive = true;
    private boolean enableLimelightDriving = true;

    // shooting systems
    private boolean enableIntake = false;
    private boolean enableTransfer = false;
    private boolean enableOutput = false;
    private boolean enableLimelightShooting = false;

    // hang systems
    private boolean enableHang = false;

    // Limelight driving
    private String limelightDrivingId = "limelight";
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;

    // Limelight shooting
    private boolean useShootingLimelight = false;
    private String limelightShootingId = "limelight2";

    // Drive speed limits
    double limitTurnSpeed = 0.85; // 75% wasn't enough, 85% seems to be about right for turning
    double limitDriveSpeed = 0.75;

    // Joystick
    Joystick joystick;
    private int joystickButtonIntake = 1;
    private int joystickButtonExtendArms = 3;
    private int joystickButtonCommenceHang = 4;
    private int joystickButtonTransfer = 5;
    private int joystickButtonDrivingAuto = 7;
    private int joystickButtonDrivingSeekBlue = 8;
    private int joystickButtonDrivingSeekRed = 9;

    // Drive Motor Controllers
    MotorControllerGroup rightBank;
    MotorControllerGroup leftBank;
    DifferentialDrive myDrive;

    // Drive Practice Controllers
    private PWMSparkMax drivePracticeLeftController;
    private PWMSparkMax drivePracticeRightController;

    // Climb Motor Controllers
    CANSparkMax frontLeftClimb;
    CANSparkMax frontRightClimb;
    CANSparkMax backLeftClimb;
    CANSparkMax backRightClimb;
    Spark leadScrews;

    // Outtake Motor Controllers
    Spark highOuttake;
    Spark lowOuttake;
    Spark transferToOuttake;
    Spark outtakeRotator;

    // Intake Motor Controllers
    Spark intakeBrush;
    Spark intakeComp;

    // Encoders
    Encoder leadScrewsEncoder;
    RelativeEncoder frontLeftClimbEncoder;
    RelativeEncoder frontRightClimbEncoder;
    RelativeEncoder backLeftClimbEncoder;
    RelativeEncoder backRightClimbEncoder;

    double leadScrewsEncoderValue;
    double frontLeftClimbEncoderValue;
    double frontRightClimbEncoderValue;
    double backLeftClimbEncoderValue;
    double backRightClimbEncoderValue;

    double transferToOuttakePower = -0.6;

    final int WIDTH = 640;

    @Override
    public void robotInit() {
        // which platform are we using?
        if (platformCurrent == platformMain) {
            System.out.println("Using robot platform MAIN");
            initMainRobot();
        } else if (platformCurrent == platformDrivePractice) {
            System.out.println("Using robot platform DRIVE PRACTICE");
            initDrivePracticeRobot();
        }

        // Joystick
        joystick = new Joystick(0);

        logDisabledSystems();

        // seek Blue by default
        if (enableDrive && enableLimelightDriving) {
            System.out.println("Limelight seeking Blue");
            NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("pipeline")
                    .setNumber(limelight_pipeline_blue);
        }
    }

    @Override
    public void robotPeriodic() {
        // only the Main robot has encoders
        if (platformMain == platformCurrent) {
            // leadScrewsEncoderValue += resetEncoderValue(leadScrewsEncoder);
            frontLeftClimbEncoderValue += resetEncoderValue(frontLeftClimbEncoder);
            frontRightClimbEncoderValue += resetEncoderValue(frontRightClimbEncoder);
            backLeftClimbEncoderValue += resetEncoderValue(backLeftClimbEncoder);
            backRightClimbEncoderValue += resetEncoderValue(backRightClimbEncoder);
        }
    }

    @Override
    public void autonomousInit() {
        // start = System.currentTimeMillis();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        // start = System.currentTimeMillis();

        logDisabledSystems();
    }

    @Override
    public void teleopPeriodic() {
        detectLimelightDriverMode();

        runDrive();

        runIntake();

        runOutput();

        runTransfer();

        runHang();
    }

    @Override
    public void testPeriodic() {
    }

    private double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
    }

    private double calcDriveCurve(double power) {
        double harshness = 8.0;

        if (power >= 1.0) {
            power = 0.99;
        }
        if (power <= -1.0) {
            power = -0.99;
        }
        if (power == 0.0) {
            return 0.0;
        }
        if (power > 0.0) {
            return Math.min(1.0, Math.max(0.0, -1 * (Math.log(1 / (power + 0) - 1) / harshness) + 0.5));
        }
        if (power < 0.0) {
            return Math.max(-1.0, Math.min(0.0, -1 * (Math.log(1 / (power + 1) - 1) / harshness) - 0.5));
        }
        return 0.0;
    }

    /**
     * Detects which mode the Limelight is in (blue vs red)
     */
    private void detectLimelightDriverMode() {
        if (enableLimelightDriving) {
            // Toggle seeking red or blue balls
            if (joystick.getRawButtonPressed(joystickButtonDrivingSeekBlue)) {
                System.out.println("Seeking Blue");
                NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("pipeline")
                        .setNumber(limelight_pipeline_blue);
            }

            if (joystick.getRawButtonPressed(joystickButtonDrivingSeekRed)) {
                System.out.println("Seeking Red");
                NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("pipeline")
                        .setNumber(limelight_pipeline_red);
            }
        }
    }

    /**
     * Handles teleoperated driving mode.
     */
    private void teleopDriver() {
        double turnValue = joystick.getRawAxis(2) * limitTurnSpeed;
        if (Math.abs(turnValue) < 0.1) {
            turnValue = 0.0;
        }

        double joystickLValue = Math.min(1.0, Math.max(-1.0, (-joystick.getRawAxis(1) + (turnValue))));
        double joystickRValue = Math.min(1.0, Math.max(-1.0, (-joystick.getRawAxis(1) - (turnValue))));

        double leftSpeed = calcDriveCurve(joystickLValue) * limitDriveSpeed;
        double rightSpeed = calcDriveCurve(joystickRValue) * limitDriveSpeed;

        // don't log very low values, just when the joystick is actually driving
        if (leftSpeed > 0.1 || leftSpeed < -0.1 || rightSpeed > 0.1 || rightSpeed < -0.1) {
            System.out.println("leftSpeed=" + leftSpeed + " rightSpeed=" + rightSpeed);
        }

        myDrive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Handles Limelight autopilot mode.
     */
    private void limelightAutoDriver() {
        // update the Limelight ball tracking data
        updateLimelightBallTracking();

        if (m_LimelightHasValidTarget) {
            // has a target
            System.out.println("Auto Drive=" + m_LimelightDriveCommand + " Steer=" + m_LimelightSteerCommand);
            myDrive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);

            return;
        }

        // seek a target by spinning in place
        myDrive.arcadeDrive(0.0, 0.5);
    }

    /**
     * Runs all of the Drive system operations.
     */
    private void runDrive() {
        if (!enableDrive) {
            myDrive.tankDrive(0.0, 0.0);

            return;
        }

        // look for auto-driving
        if (enableLimelightDriving && joystick.getRawButton(joystickButtonDrivingAuto)) {
            limelightAutoDriver();

            return;
        }

        teleopDriver();
    }

    /**
     * Runs all of the Output system operations.
     */
    private void runOutput() {
        if (!enableOutput) {
            return;
        }

        /*
         * if (joystick.getRawButton(1)) {
         * //myDrive.tankDrive(0, 0);
         * highOuttake.set(joystick.getRawAxis(1) * 0.5);
         * lowOuttake.set(joystick.getRawAxis(1) * 0.5);
         * }
         */
    }

    /**
     * Runs all of the Intake system operations.
     */
    private void runIntake() {
        if (!enableIntake) {
            return;
        }

        if (joystick.getRawButtonPressed(joystickButtonIntake)) {
            // TODO: This semems like maybe the wrong motor / power?
            transferToOuttake.set(transferToOuttakePower);
        }
    }

    /**
     * Runs all of the Transfer system operations.
     * 
     */
    private void runTransfer() {
        if (!enableTransfer) {
            return;
        }

        /*
         * if (joystick.getRawButton(joystickButtonTransfer)) {
         * transferToOuttake.set(-0.8);
         * }
         */
    }

    /**
     * Runs all of the Hang system operations.
     * 
     */
    private void runHang() {
        if (!enableHang) {
            return;
        }

        /*
         * if (joystick.getRawButton(joystickButtonExtendHang)) {
         * //Step 1: Extend back arms up
         * backLeftClimbEncoder.setPosition(0);
         * backLeftClimbEncoder.setPosition(0);
         * while (backLeftClimbEncoder.getPosition() < 500){
         * backLeftClimb.set(0.8);
         * }
         * while (backRightClimbEncoder.getPosition() < 500){
         * backRightClimb.set(0.8);
         * }
         * }
         * 
         * if (joystick.getRawButton(joystickButtonCommenceHang)){
         * commenceHang = true;
         * }
         * 
         * if (commenceHang) {
         * //Step 2: Move back arms down to winch them
         * if (hangStep == 0){
         * if (backLeftClimbEncoder.getPosition() > 4000 ||
         * backLeftClimbEncoder.getPosition() > 4000) {
         * if (backLeftClimbEncoder.getPosition() > 4000) { backLeftClimb.set(-0.5); }
         * if (backRightClimbEncoder.getPosition() > 4000) { backRightClimb.set(-0.5); }
         * }
         * hangStep++;
         * }
         * //Step 3: Extend front arms up
         * if (hangStep == 1){
         * frontLeftClimbEncoder.setPosition(0);
         * frontRightClimbEncoder.setPosition(0);
         * if (frontLeftClimbEncoder.getPosition() < 6000 ||
         * frontRightClimbEncoder.getPosition() < 6000){
         * if (frontLeftClimbEncoder.getPosition() < 6000){
         * frontLeftClimb.set(0.8);
         * }
         * if (frontRightClimbEncoder.getPosition() < 6000){
         * frontRightClimb.set(0.8);
         * }
         * hangStep++;
         * }
         * }
         * //Step 4 and 5: Rotate the robot + winch the front arms on to the high bar
         * if (hangStep == 2){
         * leadScrewsEncoder.reset();
         * if (leadScrewsEncoder.getDistance() < 35) {
         * leadScrews.set(0.5);
         * 
         * }
         * hangStep++;
         * }
         * 
         * //Steps 6 and 7: Rotate the robot back to an upright position + release the
         * weight on the back arms
         * if (hangStep == 3){
         * if (backLeftClimbEncoder.getPosition() < 5000 ||
         * backLeftClimbEncoder.getPosition() < 5000) {
         * if (backLeftClimbEncoder.getPosition() < 5000) { backLeftClimb.set(0.5);
         * frontLeftClimb.set(-0.5); }
         * if (backRightClimbEncoder.getPosition() < 5000) { backRightClimb.set(0.5);
         * frontLeftClimb.set(-0.5); }
         * }
         * hangStep++;
         * }
         * //Step 8: Extend the back arms so they are un-winched
         * if (hangStep == 4){
         * if (backLeftClimbEncoder.getPosition() < 6000 ||
         * backLeftClimbEncoder.getPosition() < 6000) {
         * if (backLeftClimbEncoder.getPosition() < 6000) { backLeftClimb.set(0.5); }
         * if (backRightClimbEncoder.getPosition() < 6000) { backRightClimb.set(0.5); }
         * }
         * hangStep++;
         * }
         * //Step 9: Rotate the back arms so that they are hovering over the traversal
         * bar
         * if (hangStep == 5){
         * if (leadScrewsEncoder.getDistance() > 25) {
         * leadScrews.set(-0.5);
         * }
         * hangStep++;
         * }
         * //Step 10: Slightly shrink the back arms for rotation
         * if (hangStep == 6){
         * if (backLeftClimbEncoder.getPosition() > 4000 ||
         * backRightClimbEncoder.getPosition() > 4000) {
         * if (backLeftClimbEncoder.getPosition() > 4000) { backLeftClimb.set(-0.5); }
         * if (backRightClimbEncoder.getPosition() > 4000) { backRightClimb.set(-0.5); }
         * }
         * hangStep++;
         * }
         * //Step 11: Rotate the back arms into a position to grab the traversal bar
         * (slightly lesss)
         * if (hangStep == 7){
         * if (leadScrewsEncoder.getDistance() > -20) {
         * leadScrews.set(-0.5);
         * }
         * hangStep++;
         * }
         * //Step 12: Extend the back arms to the height of the traversal bar
         * if (hangStep == 8){
         * if (backLeftClimbEncoder.getPosition() < 6000 ||
         * backLeftClimbEncoder.getPosition() < 6000) {
         * if (backLeftClimbEncoder.getPosition() < 6000) { backLeftClimb.set(0.5); }
         * if (backRightClimbEncoder.getPosition() < 6000) { backRightClimb.set(0.5); }
         * }
         * hangStep++;
         * 
         * }
         * //Step 13: Rotate the back arms so they latch on to the traversal bar
         * if (hangStep == 9){
         * if (leadScrewsEncoder.getDistance() > -28) {
         * leadScrews.set(-0.5);
         * }
         * hangStep++;
         * }
         * //Step 14: Same as step 6 to bring the robot into an upright position
         * if (hangStep == 10){
         * if (backLeftClimbEncoder.getPosition() > 5500 ||
         * backLeftClimbEncoder.getPosition() > 5500) {
         * if (backLeftClimbEncoder.getPosition() < 5000) { backLeftClimb.set(-0.5);
         * frontLeftClimb.set(0.5); }
         * if (backRightClimbEncoder.getPosition() < 5000) { backRightClimb.set(-0.5);
         * frontLeftClimb.set(0.5); }
         * }
         * hangStep++;
         * }
         * }
         */
    }

    /**
     * Runs the Limelight's ball-tracking code.
     * 
     */
    private void updateLimelightBallTracking() {
        // DON: I have no idea what these really do, so we should fiddle with them if we
        // need to. - 03/12/22
        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.06; // how hard to turn toward the target (initial 0.03)
        final double DRIVE_K = 0.26; // how hard to drive fwd toward the target (initial 0.26)
        final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast
        final double MIN_STEER = -0.7;
        final double MAX_STEER = 0.7;

        double tv = NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable(limelightDrivingId).getEntry("ta").getDouble(0);

        if (tv < 1.0) {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }

        if (steer_cmd > MAX_STEER) {
            steer_cmd = MAX_STEER;
        }

        if (steer_cmd < MIN_STEER) {
            steer_cmd = MIN_STEER;
        }

        m_LimelightSteerCommand = steer_cmd;
        m_LimelightDriveCommand = -drive_cmd;

        // System.out.println("Steering Tx=" + tx + " Steer=" + steer_cmd);
        // System.out.println("Driving Ta=" + ta + " Drive=" + drive_cmd);

        // m_LimelightDriveCommand = 0.0;
        // m_LimelightSteerCommand = 0.0;

    }

    /**
     * Initializes the Main robot (as opposed to the DrivePractice robot)
     * 
     */
    private void initMainRobot()
    {
        rightBank = new MotorControllerGroup(
                new CANSparkMax(2, MotorType.kBrushless),
                new CANSparkMax(9, MotorType.kBrushless));

        leftBank = new MotorControllerGroup(
                new CANSparkMax(1, MotorType.kBrushless),
                new CANSparkMax(4, MotorType.kBrushless));

        frontLeftClimb = new CANSparkMax(11, MotorType.kBrushless);
        frontRightClimb = new CANSparkMax(5, MotorType.kBrushless);
        backLeftClimb = new CANSparkMax(7, MotorType.kBrushless);
        backRightClimb = new CANSparkMax(3, MotorType.kBrushless);
        leadScrews = new Spark(0);

        highOuttake = new Spark(4);
        lowOuttake = new Spark(3);
        transferToOuttake = new Spark(2);
        outtakeRotator = new Spark(1);

        intakeBrush = new Spark(6);
        intakeComp = new Spark(5);

        leadScrewsEncoder = new Encoder(0, 1);
        frontLeftClimbEncoder = frontLeftClimb.getEncoder();
        frontRightClimbEncoder = frontRightClimb.getEncoder();
        backLeftClimbEncoder = backLeftClimb.getEncoder();
        backRightClimbEncoder = backRightClimb.getEncoder();

        myDrive = new DifferentialDrive(leftBank, rightBank);
    }

    /**
     * Initializes the DrivePractice robot (as opposed to the Main robot)
     * 
     */
    private void initDrivePracticeRobot()
    {
        drivePracticeRightController = new PWMSparkMax(0);
        drivePracticeLeftController = new PWMSparkMax(1);
        drivePracticeLeftController.setInverted(true);

        myDrive = new DifferentialDrive(drivePracticeLeftController, drivePracticeRightController);

        // disable the operations missing from the Drive Practice platform
        enableHang = false;
        enableIntake = false;
        enableOutput = false;
        enableTransfer = false;
    }

    /**
     * Log which systems are enabled/disabled for debugging
     */
    private void logDisabledSystems()
    {
        if (!enableDrive) {
            System.out.println("Driving DISABLED");
        }

        if (!enableHang) {
            System.out.println("Hang DISABLED");
        }

        if (!enableIntake) {
            System.out.println("Intake DISABLED");
        }

        if (!enableTransfer) {
            System.out.println("Transfer DISABLED");
        }

        if (!enableOutput) {
            System.out.println("Output DISABLED");
        }

        if (!enableLimelightDriving) {
            System.out.println("Limelight driving DISABLED");
        }

        if (!enableLimelightShooting) {
            System.out.println("Limelight shooting DISABLED");
        }
    }
}
