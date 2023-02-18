package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import java.util.TimerTask;
import java.util.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    // Joysticks
    Joystick joystick;

    // Motor Controllers
    CANSparkMax spark;
    double motorSpeed = 0.5;

    // Drive Motor Encoders
    RelativeEncoder encoder;

    // Timings
    double timeNow;
    double timeStart;

    // Called when robot is enabled
    @Override
    public void robotInit() {
        System.out.println("Initializing Robot");

        spark = new CANSparkMax(6, MotorType.kBrushless);

        spark.setIdleMode(IdleMode.kBrake);

        encoder = spark.getEncoder();

        joystick = new Joystick(0);
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
    }

    // Called periodically in teleoperated mode
    @Override
    public void teleopPeriodic() {
        if (joystick.getRawButton(1)) {
            spark.set(motorSpeed);
        } else {
            spark.set(0.0);
        }
    }

    // Called periodically in test mode
    @Override
    public void testPeriodic() {
    }
}
