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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

public class Robot extends TimedRobot {
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;

    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;
    private final JoystickButton m_stick_button_blue = new JoystickButton(joystick0, 2);
    private final JoystickButton m_stick_button_red = new JoystickButton(joystick0, 3);

    // Joysticks
    Joystick joystick0;
    Joystick joystick1;

    // Drivetrain
    DifferentialDrive myDrive;
    double leftPower = 0;
    double rightPower = 0;

    // Motors

    // Sparks
    CANSparkMax lbank;
    CANSparkMax rbank;
    CANSparkMax frontclimbl;
    CANSparkMax frontclimbr;
    CANSparkMax backclimbl;
    CANSparkMax backclimbr;

    TalonSRX intake;
    TalonSRX highOutput;
    TalonSRX intakebrush;

    Spark rotation;
    Spark lowOutput;
    Spark transferoutake;
    Spark leadscrews;
   

    // Encoders
    RelativeEncoder frontclimbl_encoder;
    RelativeEncoder frontclimbr_encoder;
    RelativeEncoder backclimbl_encoder;
    RelativeEncoder backclimbr_encoder;
    RelativeEncoder leadscrews_encoder;





    final int WIDTH = 640;
    
  
    @Override
    public void robotInit() {
        
        // Joystick
        joystick0 = new Joystick(0);
        joystick1 = new Joystick(1);

        rbank = new CANSparkMax(1, MotorType.kBrushless);
        lbank = new CANSparkMax(2, MotorType.kBrushless);
        frontclimbl = new CANSparkMax(3, MotorType.kBrushless);
        frontclimbr = new CANSparkMax(4, MotorType.kBrushless);
        backclimbl = new CANSparkMax(5, MotorType.kBrushless);
        frontclimbr = new CANSparkMax(6, MotorType.kBrushless);
        
        highOutput = new TalonSRX(7);
        intakebrush = new TalonSRX(8);
        intake = new TalonSRX(9);

        leadscrews = new Spark(0);
        rotation = new Spark(1);
        transferoutake = new Spark(2);
        lowOutput = new Spark(3);
        

        // Drive Train
        myDrive = new DifferentialDrive(lbank, rbank);

        // Encoders
        leadscrews_encoder = leadscrews.getEncoder();
        frontclimbl_encoder = frontclimbl.getEncoder();
        frontclimbr_encoder = frontclimbr.getEncoder();
        backclimbl_encoder = backclimbl.getEncoder();
        backclimbr_encoder = backclimbr.getEncoder();

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
        boolean auto = joystick0.getRawButton(1);
        lbank.setVoltage(12);
        rbank.setVoltage(12);
        long now = System.currentTimeMillis();
        // Turning speed limit
        double limitTurnSpeed = 0.75; // EDITABLE VALUE

        // Default manual Drive Values
        double joystickLValue =
                (-joystick0.getRawAxis(1) + (joystick0.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick0.getRawAxis(1) - (joystick0.getRawAxis(2) * limitTurnSpeed));

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

    

   