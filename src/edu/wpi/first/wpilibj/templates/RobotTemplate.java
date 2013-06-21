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
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.DriverStationLCD;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {

    AxisCamera camera;
    Dashboard board;
    DriverStationLCD LCD;
    RobotDrive robot;
    Joystick left;
    Joystick right;
    Jaguar flipper;
    Victor shooter1, shooter2;
    Victor leadScrew;
    Jaguar motor1, motor2, motor3, motor4;
    Timer timer;
    double startTime, timed;
    double delayStartTime;
    double delayFlipperTimer;
    double driveMagnitude, driveAngle;
    double x, y, rotation, xVal, yVal, rotationVal;
    double riseTime;
    boolean flipperOn, leadUp, leadDown, shooterOn, timedUp, timedDown;
    ColorImage fromCamera;
    BinaryImage filteredImage, threshholdImage, filteredSmallImage, convexHullImage;
    //robot initialization stuff here

    public void robotInit() {
        board = DriverStation.getInstance().getDashboardPackerLow();
        //used to send the debug data to the display
        LCD = DriverStationLCD.getInstance();
        timer = new Timer();
        //joystick initialization 
        right = new Joystick(2);
        left = new Joystick(1);
        //motor initialization
        motor1 = new Jaguar(1);
        motor2 = new Jaguar(2);

        motor3 = new Jaguar(3);
        motor4 = new Jaguar(4);

        robot = new RobotDrive(motor3, motor4, motor1, motor2);
        leadScrew = new Victor(8);
        flipper = new Jaguar(7);
        shooter2 = new Victor(6);
        shooter1 = new Victor(5);
        //camera initialization
        camera = AxisCamera.getInstance();
        camera.writeCompression(30);
        camera.writeMaxFPS(30);
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
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
        this.safetyOff();
        //double time;
        while (this.isOperatorControl() && this.isEnabled()) {
            //System.out.println("START");
            /*try{
             System.out.println("Start Image Processing");
             camera.freshImage();
             fromCamera=camera.getImage();
             threshholdImage = fromCamera.thresholdHSV(0, 255, 0, 255, 0, 255);
             filteredSmallImage = threshholdImage.removeSmallObjects(true,5 );
             filteredImage = filteredSmallImage.convexHull(true);
             System.out.println("End Image Processing");
             } catch(Exception v){
             System.out.println("Exception:"+v.getMessage());
             }*/

            x = left.getX();
            y = left.getY();
            rotation = right.getX();
            //System.out.println("joystick value"+" x:"+x+" y:"+y+" rotation:"+rotation+"\n");
            flipperOn = left.getTrigger();
            leadUp = left.getRawButton(3);
            leadDown = left.getRawButton(2);
            timedUp = left.getRawButton(8);
            timedDown = left.getRawButton(9);
            shooterOn = right.getTrigger();
            //System.out.println("Flipper:"+flipperOn+"leadUp:"+leadUp+"leadDown:"+leadDown+"timedUp:"+timedUp+"timedDown:"+timedDown+"shooterOn:"+shooterOn);

            //drive control
            xVal = (Math.abs(x) < 0.1) ? 0 : x * x;//square the value
            if (x >= 0) {//give back the sign 
                xVal = xVal;
            } else {
                xVal = -xVal;
            }
            yVal = (Math.abs(y) < 0.1) ? 0 : y * y;// square the value
            if (y >= 0) {//give back the sign 
                yVal = yVal;
            } else {
                yVal = -yVal;
            }
            rotationVal = (Math.abs(rotation) < 0.1) ? 0 : rotation * 0.6;

            double front_left = -yVal + rotationVal + xVal;
            double front_right = -yVal - rotationVal - xVal;
            double rear_left = -yVal + rotationVal - xVal;
            double rear_right = -yVal - rotationVal + xVal;
            motor1.set(front_left);
            motor2.set(front_right);
            motor3.set(rear_left);
            motor4.set(rear_right);
            double max = Math.abs(front_left);
            if (Math.abs(front_right) > max) {
                max = Math.abs(front_right);
            }
            if (Math.abs(rear_left) > max) {
                max = Math.abs(rear_left);
            }
            if (Math.abs(rear_right) > max) {
                max = Math.abs(rear_right);
            }
            if (max > 1) {
                front_left /= max;
                front_right /= max;
                rear_left /= max;
                rear_right /= max;
            }
            
            
            //flipper control 
            if (flipperOn) {
                flipper.set(-1);
                System.out.println("Flipper on");
            } else {
                flipper.set(0);
            }
            //shooter control
            if (shooterOn) {
                shooter1.set(1);
                shooter2.set(1);
                System.out.println("Shooter on");
            } else {
                shooter1.set(0);
                shooter2.set(0);
            }
            //lead screw control 
            if (leadUp) {
                leadScrew.set(0.5);
                System.out.println("Lead Screw up");

            } else {
                if (leadDown) {
                    leadScrew.set(-0.5);
                    System.out.println("Lead Screw down");
                } else {
                    leadScrew.set(0);
                }
            }
            /* if (timedUp && timed < 5000) {
             time = timer.get();
             leadScrew.set(1);
             timed = timed + (timer.get() - time);
             System.out.println("Lead Screw timed up");
             } else {
             if (timedDown && timed > 0) {
             time = timer.get();
             leadScrew.set(-1);
             timed = timed - (timer.get() - time);
             System.out.println("Lead Screw timed down");
             } else {
             leadScrew.set(0);
             }
             }*/

            //System.out.println("Mecanum feed value X= "+xVal+" Y="+yVal+" Z="+rotationVal );
            //System.out.println("END");
            Timer.delay(0.01);
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
        //does nothing
        //blah blah testing
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
