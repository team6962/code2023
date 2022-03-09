package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

public class Robot extends TimedRobot {
    // Limelight
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;

    //Joystick
    Joystick joystick;
    private final JoystickButton m_stick_button_blue = new JoystickButton(joystick, 2);
    private final JoystickButton m_stick_button_red = new JoystickButton(joystick, 3);

    // Drive power
    double leftPower = 0;
    double rightPower = 0;

    // Drive Motor Controllers
    MotorControllerGroup rightBank;
    MotorControllerGroup leftBank;
    DifferentialDrive myDrive;

    // Climb Motor Controllers
    CANSparkMax leftFrontClimb;
    CANSparkMax rightFrontClimb;
    CANSparkMax leftBackClimb;
    CANSparkMax rightBackClimb;
    Spark leadScrews;

    // Outtake Motor Controllers
    TalonSRX highOuttake;
    Spark lowOuttake;
    Spark transferToOuttake;
    Spark outtakeRotator;

    // Intake Motor Controllers
    TalonSRX intakeBrush;
    TalonSRX intakeComp;

    // Encoders
    RelativeEncoder leadScrewsEncoder;
    RelativeEncoder leftFrontClimbEncoder;
    RelativeEncoder rightFrontClimbEncoder;
    RelativeEncoder leftBackClimbEncoder;
    RelativeEncoder rightBackClimbEncoder;

    final int WIDTH = 640;
  
    @Override
    public void robotInit() {
        // Joystick
        joystick = new Joystick(0);

        rightBank = new MotorControllerGroup(
            new CANSparkMax(2, MotorType.kBrushless),
            new CANSparkMax(9, MotorType.kBrushless)
        );

        leftBank = new MotorControllerGroup(
            new CANSparkMax(1, MotorType.kBrushless),
            new CANSparkMax(4, MotorType.kBrushless)
        );
        
        leftFrontClimb = new CANSparkMax(11, MotorType.kBrushless);
        rightFrontClimb = new CANSparkMax(5, MotorType.kBrushless);
        leftBackClimb = new CANSparkMax(7, MotorType.kBrushless);
        rightBackClimb = new CANSparkMax(3, MotorType.kBrushless);
        leadScrews = new Spark(0);

        highOuttake = new TalonSRX(?);
        lowOuttake = new Spark(3);
        transferToOuttake = new Spark(2);
        outtakeRotator = new Spark(1);

        intakeBrush = new TalonSRX(?);
        intakeComp = new TalonSRX(?);
        
        myDrive = new DifferentialDrive(leftBank, rightBank);

        leadScrewsEncoder = leadScrews.getEncoder();
        leftFrontClimbEncoder = leftFrontClimb.getEncoder();
        rightFrontClimbEncoder = rightFrontClimb.getEncoder();
        leftBackClimbEncoder = leftBackClimb.getEncoder();
        rightBackClimbEncoder = rightBackClimb.getEncoder();

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
    }

    @Override
    public void robotPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
        start = System.currentTimeMillis();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        start = System.currentTimeMillis();
        
    }

    @Override
    public void teleopPeriodic() {
        if (m_stick.getRawButtonPressed(2)) {
            System.out.println("Seeking Blue");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
        }
      
        if (m_stick.getRawButtonPressed(3)) {
            System.out.println("Seeking Red");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_red);
        }
        Update_Limelight_Tracking();
        boolean auto = joystick.getRawButton(1);
        leftBank.setVoltage(12);
        rightBank.setVoltage(12);
        long now = System.currentTimeMillis();
        // Turning speed limit
        double limitTurnSpeed = 0.75; // EDITABLE VALUE

        // Default manual Drive Values
        double joystickLValue =
                (-joystick.getRawAxis(1) + (joystick.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick.getRawAxis(1) - (joystick.getRawAxis(2) * limitTurnSpeed));

        // ADDITIONAL DRIVE CODE HERE
        if (auto)
        {
            if (m_LimelightHasValidTarget)
            {
                System.out.println("Auto Drive=" + m_LimelightDriveCommand + " Steer=" + m_LimelightSteerCommand);
                m_robotDrive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
            }
            else
            {
            //System.out.println("Auto but no target! Drive=" + m_LimelightDriveCommand + " Steer=" + m_LimelightSteerCommand);
            m_robotDrive.arcadeDrive(0.0, 0.5);
            }
        }

        
        
        // Forgive a slight turn
        if (joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2) {
            joystickLValue = joystickRValue;
        }

        // Actual Drive code
        //myDrive.tankDrive(joystickLValue, joystickRValue);
        myDrive.tankDrive(-leftPower, -rightPower, false);
    }

    @Override
    public void testPeriodic() {}

    public void Update_Limelight_Tracking()
    {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.06;                    // how hard to turn toward the target (initial 0.03)
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target (initial 0.26)
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        final double MIN_STEER = -0.7;
        final double MAX_STEER = 0.7;

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
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
        if (drive_cmd > MAX_DRIVE)
        {
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

        //System.out.println("Steering Tx=" + tx + " Steer=" + steer_cmd);
        //System.out.println("Driving Ta=" + ta + " Drive=" + drive_cmd);

        //m_LimelightDriveCommand = 0.0;
        //m_LimelightSteerCommand = 0.0;
    }

    

   