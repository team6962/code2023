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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    MotorControllerGroup rightBank;
    MotorControllerGroup leftBank;
    //DifferentialDrive myDrive;

    // Climb Motor Controllers
    CANSparkMax leftFrontClimb;
    CANSparkMax rightFrontClimb;
    CANSparkMax leftBackClimb;
    CANSparkMax rightBackClimb;
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
    DutyCycleEncoder leadScrewsEncoder;
    RelativeEncoder leftFrontClimbEncoder;
    RelativeEncoder rightFrontClimbEncoder;
    RelativeEncoder leftBackClimbEncoder;
    RelativeEncoder rightBackClimbEncoder;

    double leadScrewsEncoderValue;
    double leftFrontClimbEncoderValue;
    double rightFrontClimbEncoderValue;
    double leftBackClimbEncoderValue;
    double rightBackClimbEncoderValue;

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

        highOuttake = new Spark(4);
        lowOuttake = new Spark(3);
        transferToOuttake = new Spark(2);
        outtakeRotator = new Spark(1);

        intakeBrush = new Spark(6);
        intakeComp = new Spark(5);
        //myDrive = new DifferentialDrive(leftBank, rightBank);

        leadScrewsEncoder = new DutyCycleEncoder(0);
        leftFrontClimbEncoder = leftFrontClimb.getEncoder();
        rightFrontClimbEncoder = rightFrontClimb.getEncoder();
        leftBackClimbEncoder = leftBackClimb.getEncoder();
        rightBackClimbEncoder = rightBackClimb.getEncoder();
       // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
    }

    @Override
    public void robotPeriodic() {
        //leadScrewsEncoderValue += resetEncoderValue(leadScrewsEncoder);
        leftFrontClimbEncoderValue += resetEncoderValue(leftFrontClimbEncoder);
        rightFrontClimbEncoderValue += resetEncoderValue(rightFrontClimbEncoder);
        leftBackClimbEncoderValue += resetEncoderValue(leftBackClimbEncoder);
        rightBackClimbEncoderValue += resetEncoderValue(rightBackClimbEncoder);
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
        //myDrive.tankDrive(0.1, -0.1);
        intakeBrush.set(0.2);
        intakeComp.set(0.2);
    }

    @Override
    public void teleopInit() {
        //start = System.currentTimeMillis();
        boolean commenceHang;
        int hangStep;

        
    }

    @Override
    public void teleopPeriodic() {
        //Toggle seeking red or blue balls
       /* if (m_stick.getRawButtonPressed(8)) {
            System.out.println("Seeking Blue");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
        }
      
        if (m_stick.getRawButtonPressed(9)) {
            System.out.println("Seeking Red");
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_red);
        }
        */
        Update_Limelight_Tracking();
        boolean auto = joystick.getRawButton(1);
        leftBank.setVoltage(12);
        rightBank.setVoltage(12);
        long now = System.currentTimeMillis();
        // Turning speed limit
        double limitTurnSpeed = 0.75; // EDITABLE VALUE

        //Outtake
       /* if (joystick.getRawButton(1)) {
            highOuttake.set(joystick.getRawAxis(3) * 0.5);
            lowOuttake.set(joystick.getRawAxis(3) * 0.5);
        }
        //Intake
        if (joystick.getRawButton(2)) {
            intakeComp.set(0.8);
            intakeBrush.set(0.8);
        }
        //Transfer
        if (joystick.getRawButton(5)){
            transferToOuttake.set(0.8)
        }
    */
        // Default manual Drive Values
        double joystickLValue =
                (-joystick.getRawAxis(1) + (joystick.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick.getRawAxis(1) - (joystick.getRawAxis(2) * limitTurnSpeed));

        // ADDITIONAL DRIVE CODE HERE
        /*
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
        */

        
        
        // Forgive a slight turn
        if (joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2) {
            joystickLValue = joystickRValue;
        }
       // double[] test = joystickToRPS(-joystick.getRawAxis(1), -joystick.getRawAxis(2));
        //double[] test2 = getDrivePower(test[0], test[1], 50);

        // Actual Drive code
        //myDrive.tankDrive(-leftPower, -rightPower, false);


        //Hang Code
        
        if (joystick.getRawButton(3)) {
            //Step 1: Extend back arms up
            leftBackClimbEncoder.setPosition(0);
            leftBackClimbEncoder.setPosition(0);
            while (leftBackClimbEncoder.getPosition() < 5000){
                leftBackClimb.set(0.8);
            }
            while (rightBackClimbEncoder.getPosition() < 5000){
                rightBackClimb.set(0.8);
            }
        }

        if (joystick.getRawButton(4)){
            commenceHang = true;
        }

        if (commenceHang) {
            //Step 2: Move back arms down to winch them
            if (hangStep == 0){
                if (leftBackClimbEncoder.getPosition() > 4000 || leftBackClimbEncoder.getPosition() > 4000) {
                    if (leftBackClimbEncoder.getPosition() > 4000) { leftBackClimb.set(-0.5); }
                    if (rightBackClimbEncoder.getPosition() > 4000) { rightBackClimb.set(-0.5); }
                }
                hangStep++;
            }
            //Step 3: Extend front arms up
            if (hangStep == 1){
                leftFrontClimbEncoder.setPosition(0);
                rightFrontClimbEncoder.setPosition(0);
                if (leftFrontClimbEncoder.getPosition() < 6000 || rightFrontClimbEncoder.getPosition() < 6000){
                    if (leftFrontClimbEncoder.getPosition() < 6000){
                        leftFrontClimb.set(0.8);
                    }
                    if (rightFrontClimbEncoder.getPosition() < 6000){
                        rightFrontClimb.set(0.8);
                    }
                hangStep++;
                }
            }
            //Step 4 and 5: Rotate the robot + winch the front arms on to the high bar
            if (hangStep == 2){
                leadScrewsEncoder.reset();
                if (leadScrewsEncoder.getDistance() < 35) {
                    leadScrews.set(0.5);
                
                }
                hangStep++;
            }
            
            //Steps 6 and 7:  Rotate the robot back to an upright position + release the weight on the back arms
            if (hangStep == 3){
                if (leftBackClimbEncoder.getPosition() < 5000 || leftBackClimbEncoder.getPosition() < 5000) {
                    if (leftBackClimbEncoder.getPosition() < 5000) { leftBackClimb.set(0.5); leftFrontClimb.set(-0.5); }
                    if (rightBackClimbEncoder.getPosition() < 5000) { rightBackClimb.set(0.5); leftFrontClimb.set(-0.5); }
                }
                hangStep++;
            }
            //Step 8: Extend the back arms so they are un-winched
            if (hangStep == 4){
                if (leftBackClimbEncoder.getPosition() < 6000 || leftBackClimbEncoder.getPosition() < 6000) {
                    if (leftBackClimbEncoder.getPosition() < 6000) { leftBackClimb.set(0.5); }
                    if (rightBackClimbEncoder.getPosition() < 6000) { rightBackClimb.set(0.5); }
                }
                hangStep++;
            }  
            //Step 9: Rotate the back arms so that they are hovering over the traversal bar
            if (hangStep == 5){
                if (leadScrewsEncoder.getDistance() > 25) {
                    leadScrews.set(-0.5);
                } 
                hangStep++;
            }
            //Step 10: Slightly shrink the back arms for rotation
            if (hangStep == 6){
                if (leftBackClimbEncoder.getPosition() > 4000 || rightBackClimbEncoder.getPosition() > 4000) {
                    if (leftBackClimbEncoder.getPosition() > 4000) { leftBackClimb.set(-0.5); }
                    if (rightBackClimbEncoder.getPosition() > 4000) { rightBackClimb.set(-0.5); }
                }
                hangStep++;
            }
            //Step 11: Rotate the back arms into a position to grab the traversal bar (slightly lesss)
            if (hangStep == 7){
                if (leadScrewsEncoder.getDistance() > -20) {
                    leadScrews.set(-0.5);
                } 
                hangStep++;
            } 
            //Step 12: Extend the back arms to the height of the traversal bar 
            if (hangStep == 8){
                if (leftBackClimbEncoder.getPosition() < 6000 || leftBackClimbEncoder.getPosition() < 6000) {
                    if (leftBackClimbEncoder.getPosition() < 6000) { leftBackClimb.set(0.5); }
                    if (rightBackClimbEncoder.getPosition() < 6000) { rightBackClimb.set(0.5); }
                }
                hangStep++;

            }
            //Step 13: Rotate the back arms so they latch on to the traversal bar
            if (hangStep == 9){
                if (leadScrewsEncoder.getDistance() > -28) {
                    leadScrews.set(-0.5);
                } 
                hangStep++;
            }
            //Step 14: Same as step 6 to bring the robot into an upright position
            if (hangStep == 10){
                if (leftBackClimbEncoder.getPosition() > 5500 || leftBackClimbEncoder.getPosition() > 5500) {
                    if (leftBackClimbEncoder.getPosition() < 5000) { leftBackClimb.set(-0.5); leftFrontClimb.set(0.5); }
                    if (rightBackClimbEncoder.getPosition() < 5000) { rightBackClimb.set(-0.5); leftFrontClimb.set(0.5); }
                }
                hangStep++;
            }
        }
        

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

       /* double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
*/
       /* if (tv < 1.0)
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
        */
    }
}
    /*public double[] joystickToRPS(double lateral, double rotational){
        double leftRotationSpeed = 5*lateral - (((Math.abs(rotational) < 0.2) ? 0 : (rotational/Math.abs(rotational))*(Math.abs(rotational)-0.2))/2);
        double rightRotationSpeed = 5*lateral + (((Math.abs(rotational) < 0.2) ? 0 : (rotational/Math.abs(rotational))*(Math.abs(rotational)-0.2))/2);
        if((leftRotationSpeed < 0.1 && rightRotationSpeed <0.1) && (leftRotationSpeed > -0.1 && rightRotationSpeed > -0.1)) return new double[]{0,0};
        return new double[]{leftRotationSpeed,rightRotationSpeed};
    }

    public double sigmoid(double x){
        return (1/(1+Math.exp(x))-0.5)*2.0;
    }

    public double[] getDrivePower(double leftRotationSpeed, double rightRotationSpeed, double div) {
        double encoder1RotationSpeed = (encoderChange[0] / 256) / (System.currentTimeMillis() - encoderChangeTime) * 1000; // rotations per sec
        double encoder2RotationSpeed = (encoderChange[1] / 256) / (System.currentTimeMillis() - encoderChangeTime) * 1000; // rotations per sec
        double leftPowerOutput = leftPower;
        double rightPowerOutput = rightPower;
        if(Math.abs(leftRotationSpeed-encoder1RotationSpeed) >= 1){
          leftPowerOutput = approx(leftRotationSpeed);
          rightPowerOutput = approx(rightRotationSpeed);
        }else{
          leftPowerOutput += sigmoid(leftRotationSpeed-encoder1RotationSpeed)/div;
          rightPowerOutput += sigmoid(rightRotationSpeed-encoder2RotationSpeed)/div;
        }
        return new double[]{leftPowerOutput, rightPowerOutput};
      }
*/
    

   
