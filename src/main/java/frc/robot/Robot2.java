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
    PWMSparkMax lbank;
    PWMSparkMax rbank;
    Spark frontclimbl;
    Spark frontclimbr;
    Spark backclimbl;
    Spark backclimbr;
    Spark intake;
    Spark rotation;
    Spark output;
    Spark intakebrush1;
    Spark intakebrush2;
    Spark transfer;

   double x = ;
   double y = 0;
   double radius = 0;
   double height = 0;

    // Encoders
    Encoder encoder1;
    Encoder encoder2;

    // Limit Switches
    DigitalInput drawerIn;
    DigitalInput drawerOut;
    DigitalInput startBelt;
    DigitalInput stopBelt;

    // Gun Values
    long startDelay = 0; // EDITABLE VALUE!!!
    boolean pullIn = false;
    boolean runIntake = true;

    // Autonomous
    ArrayList<double[]> path = new ArrayList<double[]>();
    double maxDeficit = 50;
    double maxSpeed = 0.5 + 0.2;
    int clock = 100;
    long start;
    double[] previousEncoders;
    double[] encoderBuffers;
    double[] robotVector = new double[2];
    double[] encoderChange = new double[2];
    long encoderChangeTime = 0;
    
    // Parth reinvents the wheel
    ArrayList<double[]> parth = new ArrayList<double[]>();
    int pindex = 0;
    
    final int WIDTH = 640;
    
    // Ball tracking variables
    int frames = -1;
    boolean creep = false;

    @Override
    public void robotInit() {
        
        // Joystick
        joystick0 = new Joystick(0);
        joystick1 = new Joystick(1);

        rbank = new Spark(0);
        lbank = new Spark(1);
        frontclimbl = new Spark(2);
        frontclimbr = new Spark(3);
        backclimbl = new Spark(4);
        frontclimbr = new Spark(5);
        rotation = new Spark(6);
        output = new Spark(7);
        transfer = new Spark(8);
        intakebrush1 = new Spark(9);
        intakebrush2 = new Spark(10);
        intake = new Spark(11);

        // Drive Train
        myDrive = new DifferentialDrive(lbank, rbank);

        // Encoders
        encoder1 = new Encoder(0, 1); // Left Encoder
        encoder2 = new Encoder(2, 3); // Right Encoder
        previousEncoders = new double[]{0.0, 0.0};
        encoderBuffers = new double[]{0.0, 0.0};

        // Limit Switches
        drawerIn = new DigitalInput(9);
        drawerOut = new DigitalInput(8);
        startBelt = new DigitalInput(7);
        stopBelt = new DigitalInput(6);

    }

    @Override
    public void robotPeriodic() {
        encoderChange = new double[]{getEncoder1() - previousEncoders[0], getEncoder2() - previousEncoders[1]};
        encoderChangeTime = System.currentTimeMillis();
        resetEncoders();
        robotVector[0] = (getEncoder1() - getEncoder2()) * 0.0918;
        robotVector[1] += ((encoderChange[0] / 256 * 18.85) + (encoderChange[1] / 256 * 18.85)) / 2;
        previousEncoders = new double[]{getEncoder1(), getEncoder2()};
        SmartDashboard.putNumber("Heading", robotVector[0]);
        SmartDashboard.putNumber("Distance", robotVector[1]);
    }


    //Calculate RPS from heading
    public double[] headingToRPS(double dh, double dd){

        return new double[]{(dd+(dh/0.1918)/256), (dd+(dh/0.1918)/256)};

    }

    public double getEncoder1() {
        return encoder1.getDistance() + encoderBuffers[0];
      }
  
      public double getEncoder2() {
        return encoder2.getDistance() + encoderBuffers[1];
      }

    public void resetEncoders() {
      double encoderDelta = getEncoder1() - getEncoder2();
      encoder1.reset();
      encoder2.reset();
      encoderBuffers = new double[]{0.0, 0.0};
      if (Math.abs(encoderDelta) == encoderDelta) { // If delta is positive
        encoderBuffers[0] = encoderDelta % 1877.33;
      } else {
        encoderBuffers[1] = (-encoderDelta) % 1877.33;
      }
    }

    @Override
    public void autonomousInit() {
        start = System.currentTimeMillis();
        encoder1.reset();
        encoder2.reset();
    }

    @Override
    public void autonomousPeriodic() {
        //Constants for tempering values
        double C1 = 0.005;
        double C2 = 0.005;
        long now = (System.currentTimeMillis() - start);
        //Set left and right RPM values.
        if (pindex < parth.size()){
            double lrps = parth.get(pindex)[0]+(parth.get(pindex)[3]-robotVector[1])*C1+(parth.get(pindex)[2]-robotVector[0])*C2;
            double rrps = parth.get(pindex)[1]+(parth.get(pindex)[3]-robotVector[1])*C1-(parth.get(pindex)[2]-robotVector[0])*C2;
        }
        if (now < 250){
            rbank.set(0.2);
            lbank.set(0.2);
        }
        else{
            rbank.set(0);
            lbank.set(0);
        }
        pindex++;
    }

    @Override
    public void teleopInit() {
        start = System.currentTimeMillis();
        encoder1.reset();
        encoder2.reset();
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

        // POWER CELL PICKUP CODE.
        if(joystick1.getRawButton(2)) {
            double xPos = SmartDashboard.getNumber("ballx", -1);
            double dist = SmartDashboard.getNumber("balldist", -1);
            double confidence = SmartDashboard.getNumber("confidence", 0);

            if(xPos != -1 && Math.abs(confidence) > 0.05) {
                // range: 300 - 525
                if(xPos < 400) {
                    // the ball is to the left of the frame
                    joystickLValue = -0.60;
                    joystickRValue = 0.60;
                }
                else if(300 <= xPos && xPos <= 525) {
                    // acceptable range
                    joystickLValue = 0;
                    joystickRValue = 0;
                }
                else if(xPos > 600) {
                    // the ball is to the right of the frame
                    joystickLValue = 0.60;
                    joystickRValue = -0.60;
                }
            }
        }
        // Gun
        // Transfer
        if (joystick0.getRawButton(2)) {
            runIntake = false;
        } else if (joystick0.getRawButton(3)) {
            runIntake = true;
        }

        SmartDashboard.putBoolean("canTransfer?", startBelt.get());
        if (!startBelt.get()) {

            transfer.set(-1);
            startDelay = System.currentTimeMillis();

        } else if (!stopBelt.get()) transfer.set(0);
        if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
        else intake.set(ControlMode.PercentOutput, 0);

        if (!(transfer.get() == 0)) intake.set(ControlMode.PercentOutput, -0.5);
        if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
        else intake.set(ControlMode.PercentOutput, 0);

        // Outtake
        if (joystick0.getRawButton(1)) {

            intake.set(ControlMode.PercentOutput, 0);
            transfer.set(-1);

        } else {
            if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
            else intake.set(ControlMode.PercentOutput, 0);
            transfer.set(0);
        }
        outtake.set(-(joystick0.getRawAxis(3) - 1) / 2);

        // Drawer +ve = in && -ve = out
        if (joystick0.getRawButton(7)) pullIn = false;
        if (joystick0.getRawButton(8)) pullIn = true;
        if (pullIn && drawerIn.get()) drawer.set(0.7);
        else if (!pullIn && drawerOut.get()) drawer.set(-0.7);
        else drawer.set(0);

        // Forgive a slight turn
        if (joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2) {
            joystickLValue = joystickRValue;
        }

        double[] test = joystickToRPS(-joystick0.getRawAxis(1), -joystick0.getRawAxis(2));
        double[] test2 = getDrivePower(test[0], test[1], 50);
        leftPower = test2[0];
        rightPower = test2[1];
        // Actual Drive code
        //myDrive.tankDrive(joystickLValue, joystickRValue);
        myDrive.tankDrive(-leftPower, -rightPower, false);
        // Recording RPM Values and Heading
        double[] temporary = {
            joystickLValue, joystickRValue, robotVector[0], robotVector[1], now
        };
        parth.add(temporary);
    }

    @Override
    public void testPeriodic() {}

    public double approx(double speed){
        if(speed < 1){
            return 0.6;
        }else if(speed < 2){
            return 0.7;
        }else if (speed < 3){
            return 0.75;
        }else if (speed < 4){
            return 0.8;
        }else if (speed < 5){
            return 0.85;
        }else{
            return 0;
        }
    }

    public double[] joystickToRPS(double lateral, double rotational){
        double leftRotationSpeed = 5*lateral - (((Math.abs(rotational) < 0.2) ? 0 : (rotational/Math.abs(rotational))*(Math.abs(rotational)-0.2))/2);
        double rightRotationSpeed = 5*lateral + (((Math.abs(rotational) < 0.2) ? 0 : (rotational/Math.abs(rotational))*(Math.abs(rotational)-0.2))/2);
        if((leftRotationSpeed < 0.1 && rightRotationSpeed <0.1) && (leftRotationSpeed > -0.1 && rightRotationSpeed > -0.1)) return new double[]{0,0};
        return new double[]{leftRotationSpeed,rightRotationSpeed};
    }

    public double sigmoid(double x){
        return (1/(1+Math.exp(x))-0.5)*2.0;
    }
    public double  relpos(double x, double y, double radius){
        

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

}