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

enum HM {
    NONE,
    UP,
    DOWN
}

class HangStep {
    public HM frontBar;
    public HM backBar;
    public HM leadScrew;

    public double frontBarPos;
    public double backBarPos;
    public double leadScrewPos;

    /* if true stop for user key at end of this step */
    public boolean stop;

    public HangStep(HM frontBarVal, double frontBarPosVal,
            HM backBarVal, double backBarPosVal,
            HM leadScrewVal, double leadScrewPosVal,
            boolean stopVal) {
        frontBar = frontBarVal;
        backBar = backBarVal;
        leadScrew = leadScrewVal;
        frontBarPos = frontBarPosVal;
        backBarPos = backBarPosVal;
        leadScrewPos = leadScrewPosVal;
        stop = stopVal;
    }
}

public class Robot extends TimedRobot {
    // supported platforms (Main robot and DrivePractice robot)
    private int platformMain = 1;
    private int platformDrivePractice = 2;

    // current platform for build
    private int platformCurrent = platformMain;

    // driving systems
    private boolean enableDrive = false;
    private boolean enableLimelightDriving = false;

    // shooting systems
    private boolean enableIntake = true;
    private boolean enableTransfer = true;
    private boolean enableOutput = true;
    private boolean enableLimelightShooting = false;

    // hang systems
    private boolean enableHang = false;

    // Limelight driving
    private String limelightDrivingId = "limelight-drive";
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;

    // Limelight shooting
    private boolean useShootingLimelight = false;
    private String limelightShootingId = "limelight-shoot";

    // Joystick
    // TODO: Currently, for debugging, Hang uses all of its own buttons and
    // overrides these
    Joystick joystick;
    private int joystickButtonOutput = 1;
    private int joystickButtonIntake = 2;
    private int joystickButtonExtendArms = 3;
    private int joystickButtonCommenceHang = 4;
    private int joystickButtonTransfer = 5;
    private int joystickButtonDrivingAuto = 7;
    private int joystickButtonDrivingSeekBlue = 8;
    private int joystickButtonDrivingSeekRed = 9;

    // Drive speed limits
    double limitForwardBackSpeed = 0.90; 
    double limitTwistSpeed = 0.50;

    // Hang
    boolean commenceHang;
    int hangStep = 0;
    boolean hangStepDone = true;
    HangStep[] hangSteps = {
            /* initial arm raise, set back bar to height of 350 */
            new HangStep(HM.NONE, 0, HM.UP, 380, HM.NONE, 0, true),
            /* first lift, list back bar to height of 20 */
            new HangStep(HM.NONE, 0, HM.DOWN, 5, HM.NONE, 0, true),
            /* rotate to allow high hang grab, rotate using lead screw */
            // Change 400
            new HangStep(HM.UP, 370, HM.NONE, 0, HM.DOWN, -70, true),
            // Rotate a little more (10 ticks)
            // Change 120
            new HangStep(HM.NONE, 0, HM.NONE, 0, HM.DOWN, -110, true),
            // Retract front bar (50 ticks)
            // Change 350
            new HangStep(HM.DOWN, 350, HM.NONE, 0, HM.NONE, 0, true),
            // Go to zero
            // Extend Back
            new HangStep(HM.NONE, 0, HM.UP, 300, HM.NONE, 0, true),
            // Retract Back to 50
            new HangStep(HM.DOWN, 100, HM.DOWN, 50, HM.NONE, 0, true),
            // Lead Screw goes to maxium
            new HangStep(HM.NONE, 0, HM.NONE, 0, HM.UP, 100, true),
            // rear to 380
            new HangStep(HM.NONE, 0, HM.UP, 440, HM.NONE, 0, true),
            // Lead screw to 170
            new HangStep(HM.NONE, 0, HM.NONE, 0, HM.UP, 170, true),
            //
            new HangStep(HM.NONE, 0, HM.DOWN, 320, HM.UP, 0, true),

            new HangStep(HM.UP, 380, HM.NONE, 0, HM.NONE, 0, true),
            new HangStep(HM.DOWN, 0, HM.DOWN, 0, HM.DOWN, 0, true),
            new HangStep(HM.NONE, 0, HM.NONE, 0, HM.UP, 0, true),

    };

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
    private double highOuttakePower = 0.50;
    private double lowOuttakePower = 0.75;
    private double transferToOuttakePower = 0.8;

    // Intake Motor Controllers
    Spark intakeBrush;
    Spark intakeComp;
    private double intakeBrushPower = 1.0;
    private double intakeCompPower = 1.0;

    double start;
    boolean stopzero;
    // Encoders
    Encoder leadScrewsEncoder;
    RelativeEncoder frontLeftClimbEncoder;
    RelativeEncoder frontRightClimbEncoder;
    RelativeEncoder backLeftClimbEncoder;
    RelativeEncoder backRightClimbEncoder;

    double leadScrewPos;
    double frontBarLPos;
    double frontBarRPos;
    double backBarLPos;
    double backBarRPos;

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
        start = System.currentTimeMillis();
        boolean commenceHang;

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
     * 
     * @todo Currently janky, needs testing and fine-tuning.
     */
    private void teleopDriver() {
        double rawAxisTwist = joystick.getRawAxis(2);
        double rawAxisForwardBack = -joystick.getRawAxis(1);
        double fowardBackValue = rawAxisForwardBack * limitForwardBackSpeed;

        if (Math.abs(fowardBackValue) < 0.1) {
            fowardBackValue = 0.0;
        }

        if (fowardBackValue > 0.1) {
            System.out.println("Turn Raw=" + rawAxisForwardBack + " Value=" + fowardBackValue);
        }

        double joystickLValue = rawAxisTwist + fowardBackValue;
        double joystickRValue = rawAxisTwist - fowardBackValue;

        // TODO: Apply curves?
        double leftSpeed = joystickLValue * limitTwistSpeed;
        double rightSpeed = joystickRValue * limitTwistSpeed;

        // don't log very low values, just when the joystick is actually driving
        if (leftSpeed > 0.1 || leftSpeed < -0.1 || rightSpeed > 0.1 || rightSpeed < -0.1) {
            System.out.println("rawAxisDrive=" + rawAxisTwist + " joystickLValue=" + joystickLValue + " leftSpeed=" + leftSpeed + " joystickRValue=" + joystickRValue + " rightSpeed=" + rightSpeed);
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

        // toggle on/off based on holding output button
        if (joystick.getRawButton(joystickButtonOutput)) {
            highOuttake.set(highOuttakePower);
            lowOuttake.set(lowOuttakePower);

            return;
        } 

        highOuttake.set(0.0);
        lowOuttake.set(0.0);
    }

    /**
     * Runs all of the Intake system operations.
     */
    private void runIntake() {
        if (!enableIntake) {
            return;
        }

        runIntakeCompression();
        runIntakeBrush();
    }

    private void runIntakeCompression() {
        if (!enableIntake) {
            return;
        }

        boolean running = false;

        // toggle on/of based on button press
        if (joystick.getRawButtonPressed(joystickButtonIntake)) {
            if (!running) {
                intakeComp.set(intakeCompPower);
                running = true;

                return;
            }

            intakeComp.set(0.0);
            running = false;
        }
    }

    private void runIntakeBrush() {
        if (!enableOutput) {
            return;
        }

        // toggle on/off based on holding output button
        if (joystick.getRawButton(joystickButtonOutput)) {
            intakeBrush.set(intakeBrushPower);
         
            return;
        }

        intakeBrush.set(0.0);
    }

    /**
     * Runs all of the Transfer system operations.
     * 
     */
    private void runTransfer() {
        if (!enableTransfer) {
            return;
        }

        // toggle on/off based on holding output button
        if (joystick.getRawButton(joystickButtonOutput)) {
            transferToOuttake.set(transferToOuttakePower);

            return;
        }

        transferToOuttake.set(0.0);
    }

    /**
     * Runs all of the Hang system operations.
     * 
     */
    private void runHang() {
        if (!enableHang) {
            return;
        }

        double hangspeed = 0.5;
        double leadspeed = 0.4;
        leadScrewPos = -leadScrewsEncoder.getDistance();
        frontBarLPos = frontLeftClimbEncoder.getPosition();
        frontBarRPos = frontRightClimbEncoder.getPosition();
        backBarLPos = backLeftClimbEncoder.getPosition();
        backBarRPos = backRightClimbEncoder.getPosition();
        double frontBarLSpeed = 0;
        double frontBarRSpeed = 0;
        double backBarLSpeed = 0;
        double backBarRSpeed = 0;
        double leadScrewSpeed = 0;

        // disable driving, but keep updating myDrive to squelch WPI errors in the
        // console
        // myDrive.tankDrive(0.0, 0.0);

        /*
         * double frontRightEncoderValue = frontBarL.getPosition();
         * double frontBarRPos = frontBarR.getPosition();
         * double backBarLPos = backBarL.getPosition();
         * double backBarRPos = backBarR.getPosition();
         * double leadScrewPos = leadScrew.getPosition();
         */
        // Front left forward
        if (joystick.getRawButton(3)) {
            frontBarLSpeed = 0.4;
            System.out.print(frontBarLPos);
        }
        // Front left backward
        if (joystick.getRawButton(4)) {
            frontBarLSpeed = -0.4;
            System.out.print(frontBarLPos);
        }
        // Front Right Forward
        if (joystick.getRawButton(5)) {
            frontBarRSpeed = 0.4;
            System.out.print(frontBarRPos);
        }
        // Front Right Back
        if (joystick.getRawButton(6)) {
            frontBarRSpeed = -0.4;
            System.out.print(frontBarRSpeed);
        }
        // Back Left Forward
        if (joystick.getRawButton(7)) {
            backBarLSpeed = 0.4;
            System.out.print(backBarLPos);
        }
        // Back Left Back
        if (joystick.getRawButton(8)) {
            backBarLSpeed = -0.4;
            System.out.print(backBarLPos);
        }
        // Back Right Forward
        if (joystick.getRawButton(9)) {
            backBarRSpeed = 0.4;
            System.out.print(backBarRPos);
        }
        // Back Right Back
        if (joystick.getRawButton(10)) {
            backBarRSpeed = -0.4;
            System.out.print(backBarRPos);
        }
        // Lead Screw Forward
        if (joystick.getRawButton(1)) {
            leadScrewSpeed = 0.4;
            System.out.print(leadScrewPos);
        }
        // Lead Screw Backwards
        if (joystick.getRawButton(2)) {
            leadScrewSpeed = -0.4;
            System.out.print(leadScrewPos);
        }

        // Next Step
        if (hangStepDone && joystick.getRawButtonPressed(11)) {
            hangStepDone = false;
            System.out.print("Button 11, hangstepdone: " + hangStepDone + " Hangstep: " + hangStep);
        }
        // Kill Switch
        if (joystick.getRawButtonPressed(12)) {
            System.out.print("Backleft: " + backBarLPos + "\n");
            System.out.print("Back Right: \n" + backBarRPos + "\n");
            System.out.print("Front Left: \n" + frontBarLPos + "\n");
            System.out.print("Front Right: \n" + frontBarRPos + "\n");
            System.out.print("Lead Screw: \n" + leadScrewPos + "\n");
            hangStepDone = true;
        }

        if (!hangStepDone && hangStep < hangSteps.length) {
            boolean stepDone = true;

            /* front bar left */
            if (hangSteps[hangStep].frontBar == HM.UP && frontBarLPos < hangSteps[hangStep].frontBarPos) {
                frontBarLSpeed = hangspeed;
                stepDone = false;
            } else if (hangSteps[hangStep].frontBar == HM.DOWN && frontBarLPos > hangSteps[hangStep].frontBarPos) {
                frontBarLSpeed = -hangspeed;
                stepDone = false;
            }

            /* front bar right */
            if (hangSteps[hangStep].frontBar == HM.UP && frontBarRPos < hangSteps[hangStep].frontBarPos) {
                frontBarRSpeed = hangspeed;
                stepDone = false;
            } else if (hangSteps[hangStep].frontBar == HM.DOWN && frontBarRPos > hangSteps[hangStep].frontBarPos) {
                frontBarRSpeed = -hangspeed;
                stepDone = false;
            }

            /* back bar left */
            if (hangSteps[hangStep].backBar == HM.UP && backBarLPos < hangSteps[hangStep].backBarPos) {
                backBarLSpeed = hangspeed;
                stepDone = false;
            } else if (hangSteps[hangStep].backBar == HM.DOWN && backBarLPos > hangSteps[hangStep].backBarPos) {
                backBarLSpeed = -hangspeed;
                stepDone = false;
            }

            /* back bar right */
            if (hangSteps[hangStep].backBar == HM.UP && backBarRPos < hangSteps[hangStep].backBarPos) {
                backBarRSpeed = hangspeed;
                stepDone = false;
            } else if (hangSteps[hangStep].backBar == HM.DOWN && backBarRPos > hangSteps[hangStep].backBarPos) {
                backBarRSpeed = -hangspeed;
                stepDone = false;
            }

            /* lead screw */
            if (hangSteps[hangStep].leadScrew == HM.UP && leadScrewPos < hangSteps[hangStep].leadScrewPos) {
                leadScrewSpeed = leadspeed;
                stepDone = false;
            } else if (hangSteps[hangStep].leadScrew == HM.DOWN && leadScrewPos > hangSteps[hangStep].leadScrewPos) {
                leadScrewSpeed = -leadspeed;
                stepDone = false;
            }

            if (stepDone) {
                hangStepDone = hangSteps[hangStep].stop;
                hangStep++;
            }

        }

        // Next 5 statements prevent robot going beyond
        // mechanical limits, DO NOT CHNGE
        /*
         * if (frontBarLPos > 450 || frontBarLPos < 0) {
         * frontBarLSpeed = 0;
         * }
         * if (frontBarRPos > 455 || frontBarRPos < 0) {
         * frontBarRSpeed = 0;
         * }
         * if (backBarLPos > 440 || backBarLPos < 0) {
         * backBarLSpeed = 0;
         * }
         * if (backBarRPos > 455 || backBarRPos < 0) {
         * backBarRSpeed = 0;
         * }
         * if (leadScrewPos > 170 || backBarRPos < -115) {
         * leadScrewSpeed = 0;
         * }
         */

        frontLeftClimb.set(frontBarLSpeed);
        frontRightClimb.set(frontBarRSpeed);
        backLeftClimb.set(backBarLSpeed);
        backRightClimb.set(backBarRSpeed);
        leadScrews.set(leadScrewSpeed);
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
    private void initMainRobot() {
        // drive motors
        rightBank = new MotorControllerGroup(
                new CANSparkMax(2, MotorType.kBrushless),
                new CANSparkMax(9, MotorType.kBrushless));

        leftBank = new MotorControllerGroup(
                new CANSparkMax(1, MotorType.kBrushless),
                new CANSparkMax(4, MotorType.kBrushless));

        myDrive = new DifferentialDrive(leftBank, rightBank);

        // climb motors
        frontLeftClimb = new CANSparkMax(11, MotorType.kBrushless);
        frontRightClimb = new CANSparkMax(5, MotorType.kBrushless);
        backLeftClimb = new CANSparkMax(7, MotorType.kBrushless);
        backRightClimb = new CANSparkMax(3, MotorType.kBrushless);
        leadScrews = new Spark(0);

        // outtake motors
        outtakeRotator = new Spark(1);
        highOuttake = new Spark(4);
        lowOuttake = new Spark(3);
        lowOuttake.setInverted(true);

        // transfer motors
        transferToOuttake = new Spark(2);
        transferToOuttake.setInverted(true);

        // intake motors
        intakeBrush = new Spark(5);
        intakeComp = new Spark(6);
        intakeComp.setInverted(true);

        // hang encoders
        leadScrewsEncoder = new Encoder(0, 1);
        frontLeftClimbEncoder = frontLeftClimb.getEncoder();
        frontRightClimbEncoder = frontRightClimb.getEncoder();
        backLeftClimbEncoder = backLeftClimb.getEncoder();
        backRightClimbEncoder = backRightClimb.getEncoder();

        stopzero = true;

        // TODO: Currently Hang overloads all the joystick buttons for debugging,
        // disable all other systems to prevent accidents
        if (enableHang) {
            enableDrive = false;
            enableIntake = false;
            enableTransfer = false;
            enableOutput = false;
            enableLimelightDriving = false;
            enableLimelightShooting = false;

            System.out.println("Hang ENABLED, disabling ALL other systems");
            logDisabledSystems();
        }
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

        // disable the operations missing from the Drive Practice platform
        enableHang = false;
        enableIntake = false;
        enableOutput = false;
        enableTransfer = false;
    }

    /**
     * Log which systems are enabled/disabled for debugging
     */
    private void logDisabledSystems() {
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
