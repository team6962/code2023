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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

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
    // Limelight
   /*private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    private int limelight_pipeline_blue = 4;
    private int limelight_pipeline_red = 3;
*/
    //Joystick
    Joystick joystick;
    //private final JoystickButton m_stick_button_blue = new JoystickButton(joystick, 2);
    //private final JoystickButton m_stick_button_red = new JoystickButton(joystick, 3);

    // Drive Power
    double leftPower = 0;
    double rightPower = 0;

    //Hang
    boolean commenceHang;
    int hangStep = 0;
	boolean hangStepDone = true;
    HangStep[] hangSteps = {
        /* initial arm raise, set back bar to height of 350 */
        new HangStep(HM.NONE, 0, HM.UP, 380, HM.NONE, 0, true),
        /* first lift, list back bar to height of 20*/
        new HangStep(HM.NONE, 0, HM.DOWN, 5, HM.NONE, 0, true),
        /* rotate to allow high hang grab, rotate using lead screw */
        // Change 400
        new HangStep(HM.UP, 370, HM.NONE, 0, HM.DOWN, -70, true),
        //Rotate a little more (10 ticks)
        //Change 120
        new HangStep(HM.NONE, 0, HM.NONE, 0, HM.DOWN, -110, true),
        //Retract front bar (50 ticks)
        //Change 350
        new HangStep(HM.DOWN, 350, HM.NONE, 0, HM.NONE, 0, true),
        //Go to zero
        //Extend Back
        new HangStep(HM.NONE, 0, HM.UP, 300, HM.NONE, 0, true),
        //Retract Back to 50
        new HangStep(HM.DOWN, 100, HM.DOWN, 50, HM.NONE, 0, true),
        //Lead Screw goes to maxium
        new HangStep(HM.NONE, 0, HM.NONE, 0, HM.UP, 100, true),
        //rear to 380
        new HangStep(HM.NONE, 0, HM.UP, 440, HM.NONE, 0, true),
        //Lead screw to 170
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
    double transferToOuttakePower = -0.6;
    

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
        
        frontLeftClimb = new CANSparkMax(11, MotorType.kBrushless);
        frontRightClimb = new CANSparkMax(5, MotorType.kBrushless);
        backLeftClimb = new CANSparkMax(7, MotorType.kBrushless);
        backRightClimb = new CANSparkMax(3, MotorType.kBrushless);
        leadScrews = new Spark(0);

        highOuttake = new Spark(4);
        lowOuttake = new Spark(3);
        transferToOuttake = new Spark(2);
        outtakeRotator = new Spark(1);

        intakeBrush = new Spark(5);
        intakeComp = new Spark(6);
        myDrive = new DifferentialDrive(leftBank, rightBank);

        leadScrewsEncoder = new Encoder(0, 1);
        frontLeftClimbEncoder = frontLeftClimb.getEncoder();
        frontRightClimbEncoder = frontRightClimb.getEncoder();
        backLeftClimbEncoder = backLeftClimb.getEncoder();
        backRightClimbEncoder = backRightClimb.getEncoder();
       // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(limelight_pipeline_blue);
       stopzero = true;
    }

    @Override
    public void robotPeriodic() {
        //leadScrewsEncoderValue += resetEncoderValue(leadScrewsEncoder);
        //frontLeftClimbEncoderValue += resetEncoderValue(frontLeftClimbEncoder);
        //frontRightClimbEncoderValue += resetEncoderValue(frontRightClimbEncoder);
        //backLeftClimbEncoderValue += resetEncoderValue(backLeftClimbEncoder);
        //backRightClimbEncoderValue += resetEncoderValue(backRightClimbEncoder);
    }

   /* public double resetEncoderValue(RelativeEncoder encoder) {
        double value = encoder.getPosition();
        encoder.setPosition(0);
        return value;
    }
*/
    @Override
    public void autonomousInit() {
       //start = System.currentTimeMillis();
    }

    @Override
    public void autonomousPeriodic() {
        //System.out.print("here");
        //System.out.print("autonstart");
        //backRightClimb.set(-0.4);
        //backLeftClimb.set(-0.4);
        intakeComp.set(-1);
        intakeBrush.set(1);
        transferToOuttake.set(-0.8);
    }

    /*public void doIntakeTransfer() {
        transferToOuttake.set(transferToOuttakePower);
    }
    */

    @Override
    public void teleopInit() {
        start = System.currentTimeMillis();
        boolean commenceHang;
        

        
    }

 /*  public double calcDriveCurve(double power) {
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
    
*/
    @Override
    public void teleopPeriodic() {
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

        // disable driving, but keep updating myDrive to squelch WPI errors in the console
        myDrive.tankDrive(0.0, 0.0);

        /*
			double frontRightEncoderValue = frontBarL.getPosition();
			double frontBarRPos = frontBarR.getPosition();
			double backBarLPos = backBarL.getPosition();
			double backBarRPos = backBarR.getPosition();
			double leadScrewPos = leadScrew.getPosition();
			*/
        //Front left forward
        if (joystick.getRawButton(3)){
            frontBarLSpeed = 0.4;
            System.out.print(frontBarLPos);
        }
        //Front left backward
        if (joystick.getRawButton(4)){
            frontBarLSpeed = -0.4;
            System.out.print(frontBarLPos);
        }
        //Front Right Forward
        if (joystick.getRawButton(5)){
            frontBarRSpeed = 0.4;
            System.out.print(frontBarRPos);
        }
        //Front Right Back
        if (joystick.getRawButton(6)){
            frontBarRSpeed = -0.4;
            System.out.print(frontBarRSpeed);
        }
        //Back Left Forward
        if (joystick.getRawButton(7)){
            backBarLSpeed = 0.4;
            System.out.print(backBarLPos);
        }
        //Back Left Back
        if (joystick.getRawButton(8)){
            backBarLSpeed = -0.4;
            System.out.print(backBarLPos);
        }
        //Back Right Forward
        if (joystick.getRawButton(9)){
            backBarRSpeed = 0.4;
            System.out.print(backBarRPos);
        }
        //Back Right Back
        if (joystick.getRawButton(10)){
            backBarRSpeed = -0.4;
            System.out.print(backBarRPos);
        }
        //Lead Screw Forward
        if (joystick.getRawButton(1)){
            leadScrewSpeed = 0.4;
            System.out.print(leadScrewPos);
        }
        //Lead Screw Backwards
        if (joystick.getRawButton(2)){
            leadScrewSpeed = -0.4;
            System.out.print(leadScrewPos);
        }
        

        

    /*
        if (joystick.getRawButtonPressed(5)) {
            //doIntakeTransfer();
            leadScrews.set(0.5);
        } else if (joystick.getRawButtonPressed(6)) {
            leadScrews.set(-0.5);
        } else if (!joystick.getRawButton(5) && !joystick.getRawButton(6)) {
            leadScrews.set(0);
        }
    */

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
        //Update_Limelight_Tracking();
        //boolean auto = joystick.getRawButton(1);
        //long now = System.currentTimeMillis();
        // Turning speed limit
        double limitTurnSpeed = 0.5; // EDITABLE VALUE

        //Outtake
        /*if (joystick.getRawButton(1)) {
            //myDrive.tankDrive(0, 0);
            highOuttake.set(joystick.getRawAxis(1) * 0.5);
            lowOuttake.set(joystick.getRawAxis(1) * 0.5);
        }
        //Intake
        if (joystick.getRawButton(2)) {
            intakeComp.set(0.8);
            intakeBrush.set(0.8);
        }
        //Transfer
        if (joystick.getRawButton(5)) {
            transferToOuttake.set(-0.8);
        }
        */
        // Default manual Drive Values
        double joystickLValue =
                (-joystick.getRawAxis(1) + (joystick.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick.getRawAxis(1) - (joystick.getRawAxis(2) * limitTurnSpeed));

        // ADDITIONAL DRIVE CODE HERE
        
        
        // Forgive a slight turn
       // if (joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2) {
         //   joystickLValue = joystickRValue;
        //}
        //double[] test = joystickToRPS(-joystick.getRawAxis(1), -joystick.getRawAxis(2));
        //double[] test2 = getDrivePower(test[0], test[1], 50);

        // Actual Drive code
        
        //myDrive.tankDrive(joystickLValue, -joystickRValue);
       /* if (joystick.getRawButton(9) && backRightClimbEncoder.getPosition() > 0 && backLeftClimbEncoder.getPosition() > 0){
            backRightClimb.set(-1);
            backLeftClimb.set(-1);
            System.out.print("downright " + backRightClimbEncoder.getPosition());
            System.out.print("downleft " + backLeftClimbEncoder.getPosition());
        }
        else if (joystick.getRawButton(7) && backLeftClimbEncoder.getPosition() < 444 && backRightClimbEncoder.getPosition() < 444){
            backLeftClimb.set(1);
            backRightClimb.set(1);
            System.out.print("upright " + backRightClimbEncoder.getPosition());
            System.out.print("upleft " + backLeftClimbEncoder.getPosition());
        }
        else{
            backLeftClimb.set(0);
            backRightClimb.set(0);
        }
        if (joystick.getRawButtonPressed(8)){
            backLeftClimbEncoder.setPosition(0);
            backRightClimbEncoder.setPosition(0);
        }
        */
        /*
        if (joystick.getRawButton(9) && (-leadScrewsEncoder.getDistance() > -115)){
            leadScrews.set(-0.5);
            System.out.print("leadback " + -leadScrewsEncoder.getDistance());
        }
        else if (joystick.getRawButton(7) && (-leadScrewsEncoder.getDistance() < 173)){
            leadScrews.set(0.5);
            System.out.print("leadforward " + -leadScrewsEncoder.getDistance());
        }
        else{
            leadScrews.set(0);
        }
        if (joystick.getRawButtonPressed(8)){
            leadScrewsEncoder.reset();
        }
        if (joystick.getRawButtonPressed(11)){
            System.out.print("Encoder "+leadScrewsEncoder.getDistance());
        }
        if (joystick.getRawButton(12) && stopzero == true){
            if (-leadScrewsEncoder.getDistance() < 3 && -leadScrewsEncoder.getDistance() > -5){
                leadScrews.set(0);
            }
            if (-leadScrewsEncoder.getDistance() < 0){
                leadScrews.set(0.3);
            }
            else if (-leadScrewsEncoder.getDistance() > 0){
                leadScrews.set(-0.3);
            }
            if (-leadScrewsEncoder.getDistance() > -2 && -leadScrewsEncoder.getDistance() < 2){
                stopzero = false;
            }
        
                        
        }
        else if (joystick.getRawButton(12) == false){
            stopzero = true;
        }
        */
        //Hang Code
        //Next Step
        if(hangStepDone && joystick.getRawButtonPressed(11)) {
			hangStepDone = false;
            System.out.print("Button 11, hangstepdone: "+ hangStepDone + " Hangstep: " + hangStep);
		}
        //Kill Switch
        if (joystick.getRawButtonPressed(12)){
            System.out.print("Backleft: " + backBarLPos);
            System.out.print("Back Right: " + backBarRPos);
            System.out.print("Front Left: " + frontBarLPos);
            System.out.print("Front Right: " + frontBarRPos);
            System.out.print("Lead Screw: " + leadScrewPos);
            hangStepDone = true;
        }
 
		if(!hangStepDone && hangStep < hangSteps.length) {
			boolean stepDone = true;
 
			
			/* front bar left*/
			if(hangSteps[hangStep].frontBar == HM.UP && frontBarLPos < hangSteps[hangStep].frontBarPos) {
				frontBarLSpeed = hangspeed;
				stepDone = false;
			} else if (hangSteps[hangStep].frontBar == HM.DOWN && frontBarLPos > hangSteps[hangStep].frontBarPos) {
				frontBarLSpeed = -hangspeed;
				stepDone = false;
			}
 
			/* front bar right */
			if(hangSteps[hangStep].frontBar == HM.UP && frontBarRPos < hangSteps[hangStep].frontBarPos) {
				frontBarRSpeed = hangspeed;
				stepDone = false;
			} else if (hangSteps[hangStep].frontBar == HM.DOWN && frontBarRPos > hangSteps[hangStep].frontBarPos) {
				frontBarRSpeed = -hangspeed;
				stepDone = false;
			}
 
			/* back bar left */
			if(hangSteps[hangStep].backBar == HM.UP && backBarLPos < hangSteps[hangStep].backBarPos) {
				backBarLSpeed = hangspeed;
				stepDone = false;
			} else if (hangSteps[hangStep].backBar == HM.DOWN && backBarLPos > hangSteps[hangStep].backBarPos) {
				backBarLSpeed = -hangspeed;
				stepDone = false;
			}
 
			/* back bar right */
			if(hangSteps[hangStep].backBar == HM.UP && backBarRPos < hangSteps[hangStep].backBarPos) {
				backBarRSpeed = hangspeed;
				stepDone = false;
			} else if (hangSteps[hangStep].backBar == HM.DOWN && backBarRPos > hangSteps[hangStep].backBarPos) {
				backBarRSpeed = -hangspeed;
				stepDone = false;
			}
 
			/* lead screw */
			if(hangSteps[hangStep].leadScrew == HM.UP && leadScrewPos < hangSteps[hangStep].leadScrewPos) {
				leadScrewSpeed = leadspeed;
				stepDone = false;
			} else if (hangSteps[hangStep].leadScrew == HM.DOWN && leadScrewPos > hangSteps[hangStep].leadScrewPos) {
				leadScrewSpeed = -leadspeed;
				stepDone = false;
			}
 
			if(stepDone) {
				hangStepDone = hangSteps[hangStep].stop;
				hangStep++;
			}
 
		}

        // Next 5 statements prevent robot going beyond
        // mechanical limits, DO NOT CHNGE
        /*
        if (frontBarLPos > 450 || frontBarLPos < 0) {
            frontBarLSpeed = 0;
        }
        if (frontBarRPos > 455 || frontBarRPos < 0) {
            frontBarRSpeed = 0;
        }
        if (backBarLPos > 440 || backBarLPos < 0) {
            backBarLSpeed = 0;
        }
        if (backBarRPos > 455 || backBarRPos < 0) {
            backBarRSpeed = 0;
        }
        if (leadScrewPos > 170 || backBarRPos < -115) {
            leadScrewSpeed = 0;
        }*/
 
		
		frontLeftClimb.set(frontBarLSpeed);
		frontRightClimb.set(frontBarRSpeed);
		backLeftClimb.set(backBarLSpeed);
		backRightClimb.set(backBarRSpeed);
		leadScrews.set(leadScrewSpeed);
		
        
        

    }

    @Override
    public void testPeriodic() {}
}
    
