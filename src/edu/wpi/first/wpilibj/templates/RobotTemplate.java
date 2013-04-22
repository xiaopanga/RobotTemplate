/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.camera.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {

    //AxisCamera camera = AxisCamera.getInstance();
    Dashboard board = DriverStation.getInstance().getDashboardPackerLow();
    RobotDrive robot;
    Joystick left;
    Joystick right;
    Jaguar shooter1, shooter2, flipper;
    Victor leadScrew;
    Victor motor1, motor2, motor3, motor4;
    Timer timer;
    double startTime, timed;
    double delayStartTime;
    double delayFlipperTimer;
    double x, y, rotation;
    double riseTime;
    boolean flipperOn, leadUp, leadDown, shooterOn, timedUp, timedDown;
    //robot initialization stuff here

    public void robotInit() {
        timer = new Timer();
        //joystick initialization 
        right = new Joystick(2);
        left = new Joystick(1);
        //motor initialization
        motor1 = new Victor(1);
        motor2 = new Victor(2);

        motor3 = new Victor(3);
        motor4 = new Victor(4);

        robot = new RobotDrive(motor1, motor2, motor3, motor4);
        robot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        robot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        leadScrew = new Victor(8);
        flipper = new Jaguar(7);
        shooter2 = new Jaguar(6);
        shooter1 = new Jaguar(5);
        //camera initialization
        //camera.writeCompression(30);
        //camera.writeMaxFPS(30);
        //camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        // autonomous delay time in ms
        delayStartTime = 2000;
        delayFlipperTimer = 2000;
        //initial value for leadscrew rise time
        riseTime = 0;
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        this.safetyOff();
        shooter1.setSafetyEnabled(true);
        shooter2.setSafetyEnabled(true);
        flipper.setSafetyEnabled(true);
        startTime = timer.get();

        //delayStartTime = 0;//how to get value from Dashboard?
        while (this.isAutonomous() && this.isEnabled()) {
            if (startTime + delayStartTime > timer.get()) {
                shooter1.set(1);
                shooter2.set(1);
                if (startTime + delayStartTime + delayFlipperTimer > timer.get()) {
                    flipper.set(-1);
                }
            } else {
                shooter1.set(0);
                shooter2.set(0);
                flipper.set(0);
            }
            Timer.delay(0.01);

        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        this.safetyOn();
        double time;
        while (this.isOperatorControl() && this.isEnabled()) {
            x = left.getX();
            y = left.getY();
            rotation = right.getX();
            flipperOn = left.getTrigger();
            leadUp = left.getRawButton(3);
            leadDown = left.getRawButton(2);
            timedUp = left.getRawButton(8);
            timedDown = left.getRawButton(9);
            shooterOn = right.getTrigger();
            if (flipperOn) {
                flipper.set(-1);
            } else {
                flipper.set(0);
            }
            if (shooterOn) {
                shooter1.set(1);
                shooter2.set(1);
            } else {
                shooter1.set(0);
                shooter2.set(0);
            }
            if (leadUp) {
                leadScrew.set(1);
            } else {
                if (leadDown) {
                    leadScrew.set(-1);
                } else {
                    leadScrew.set(0);
                }
            }
            if (timedUp && timed < 5000) {
                time = timer.get();
                leadScrew.set(1);
                timed = timed + (timer.get() - time);
            } else {
                if (timedDown && timed > 0) {
                    time = timer.get();
                    leadScrew.set(-1);
                    timed = timed - (timer.get() - time);
                } else {
                    leadScrew.set(0);
                }
            }

            robot.mecanumDrive_Cartesian((Math.abs(x) < 0.05) ? 0 : x * x, (Math.abs(y) < 0.05) ? 0 : y * y, (Math.abs(rotation) < 0.05) ? 0 : rotation * 0.6, 0);

            Timer.delay(0.01);
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    }
    //diabled state

    public void disabled() {
        this.safetyOff();
        while (this.isDisabled()) {
            Timer.delay(0.01);
        }
    }

    public void safetyOff() {
        robot.setSafetyEnabled(false);
        shooter1.setSafetyEnabled(false);
        shooter2.setSafetyEnabled(false);
        flipper.setSafetyEnabled(false);
        leadScrew.setSafetyEnabled(false);
    }

    public void safetyOn() {
        robot.setSafetyEnabled(true);
        shooter1.setSafetyEnabled(true);
        shooter2.setSafetyEnabled(true);
        flipper.setSafetyEnabled(true);
        leadScrew.setSafetyEnabled(true);
    }
}
