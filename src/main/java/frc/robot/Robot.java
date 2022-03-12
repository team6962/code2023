package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.DutyCycleDataJNI;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

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
    //private final JoystickButton m_stick_button_blue = new JoystickButton(joystick, 2);
    //private final JoystickButton m_stick_button_red = new JoystickButton(joystick, 3);

    // Drive Power
    double leftPower = 0;
    double rightPower = 0;

    //Hang
    boolean commenceHang;
    int hangStep;

    // Drive Motor Controllers
    PWMSparkMax rightBank;
    Spark leftBank;
    DifferentialDrive myDrive;

    final int WIDTH = 640;
    
    @Override
    public void robotInit() {
        // Joystick
        joystick = new Joystick(0);

        rightBank = new PWMSparkMax(0);
        leftBank = new Spark(1);

        myDrive = new DifferentialDrive(leftBank, rightBank);

       // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
    }

    @Override
    public void robotPeriodic() {
       
    }

    public double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
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
        //start = System.currentTimeMillis();
        
    }

    public double calcDriveCurve(double power) {
        return power * Math.abs(power);
    }
    

    @Override
    public void teleopPeriodic() {
        double limitTurnSpeed = 0.75;
        double limitDriveSpeed = 0.75;

        double turnValue = joystick.getRawAxis(2) * limitTurnSpeed;
        if (Math.abs(turnValue) < 0.1) {
            turnValue = 0.0;
        }

        double joystickLValue =
            Math.min(1.0, Math.max(-1.0, (-joystick.getRawAxis(1) + (turnValue))));
        double joystickRValue =
            Math.min(1.0, Math.max(-1.0, (-joystick.getRawAxis(1) - (turnValue))));
        
        System.out.println(calcDriveCurve(joystickLValue) * limitDriveSpeed);

        myDrive.tankDrive(calcDriveCurve(joystickLValue) * limitDriveSpeed, calcDriveCurve(joystickRValue) * limitDriveSpeed);
    }

    @Override
    public void testPeriodic() {}
}