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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

private boolean m_LimelightHasValidTarget = false;
private double m_LimelightDriveCommand = 0.0;
private double m_LimelightSteerCommand = 0.0;
private int limelight_pipeline_blue = 4;
private int limelight_pipeline_red = 3;

public class Robot extends TimedRobot {

    // Joysticks
    Joystick joystick0;
    Joystick joystick1;

    // Drivetrain
    DifferentialDrive myDrive;
    double leftPower = 0;
    double rightPower = 0;

    // Motors

    // Sparks
    Spark lbank;
    Spark rbank;
    CANSparkMax frontclimbl;
    CANSparkMax frontclimbr;
    CANSparkMax backclimbl;
    CANSparkMax backclimbr;
    Spark intake;
    Spark rotation;
    Spark output;
    Spark output2;
    Spark intakebrush;
    Spark transfer;
    Spark transferoutake;
    CANSparkMax leadscrew1;
    CANSparkMax leadscrew2;

   

    // Encoders
    RelativeEncoder frontclimbl_encoder;
    RelativeEncoder frontclimbr_encoder;
    RelativeEncoder backclimbl_encoder;
    RelativeEncoder backclimbr_encoder;
    RelativeEncoder leadscrew1_encoder;
    RelativeEncoder leadscrew2_encoder;





    final int WIDTH = 640;
    
  
    @Override
    public void robotInit() {
        
        // Joystick
        joystick0 = new Joystick(0);
        joystick1 = new Joystick(1);

        rbank = new Spark(0);
        lbank = new Spark(1);
        leadscrew1 = new CANSparkMax(0, MotorType.kBrushless);
        leadscrew2 = new CANSparkMax(1, MotorType.kBrushless);
        frontclimbl = new CANSparkMax(2, MotorType.kBrushless);
        frontclimbr = new CANSparkMax(3, MotorType.kBrushless);
        backclimbl = new CANSparkMax(4, MotorType.kBrushless);
        frontclimbr = new CANSparkMax(5, MotorType.kBrushless);
        rotation = new Spark(2);
        output = new Spark(3);
        transfer = new Spark(4);
        intakebrush = new Spark(5);
        intake = new Spark(7);
        transferoutake = new Spark(8);
        output2 = new Spark(3);

        // Drive Train
        myDrive = new DifferentialDrive(lbank, rbank);

        // Encoders
        leadscrew1_encoder = leadscrew1.getEncoder();
        leadscrew2_encoder = leadscrew2.getEncoder();
        frontclimbl_encoder = frontclimbl.getEncoder();
        frontclimbr_encoder = frontclimbr.getEncoder();
        backclimbl_encoder = backclimbl.getEncoder();
        backclimbr_encoder = backclimbr.getEncoder();


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

    

   