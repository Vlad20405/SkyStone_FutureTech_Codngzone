/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Autonom-baterie-full", group="Pushbot")
//@Disabled
public class

Autonom2022 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //Motoare sasiu:
    private DcMotor Dreapta_F = null;
    private DcMotor Dreapta_S = null;
    private DcMotor Stanga_F = null;
    private DcMotor Stanga_S = null;

    //Motoare brat:
    private DcMotor Brat_M =null;
    private DcMotor Cutie=null;
    private DcMotor Carusel=null;
    //Servo-uri:
    private CRServo Colectare = null;

    //Senzor culoare:
    ColorSensor senzorCuloare;
    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        //Senzor culoare:

        senzorCuloare = hardwareMap.get(ColorSensor.class, "senzorCuloare");
/*
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (senzorCuloare.red() * SCALE_FACTOR),
                    (int) (senzorCuloare.green() * SCALE_FACTOR),
                    (int) (senzorCuloare.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.

            telemetry.addData("Alpha", senzorCuloare.alpha());
            telemetry.addData("Red  ", senzorCuloare.red());
            telemetry.addData("Green", senzorCuloare.green());
            telemetry.addData("Blue ", senzorCuloare.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run () {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }

        });
*/
        Stanga_F = hardwareMap.get(DcMotor.class, "Stanga_F");
        Stanga_S = hardwareMap.get(DcMotor.class, "Stanga_S");
        Dreapta_F = hardwareMap.get(DcMotor.class, "Dreapta_F");
        Dreapta_S = hardwareMap.get(DcMotor.class, "Dreapta_S");

        Brat_M = hardwareMap.get(DcMotor.class,"Brat_M");
        Cutie= hardwareMap.get(DcMotor.class,"Cutie");
        Carusel= hardwareMap.get(DcMotor.class,"Carusel");

        Colectare= hardwareMap.get(CRServo.class,"Colectare");

        Stanga_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stanga_S.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Dreapta_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Dreapta_S.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Stanga_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Stanga_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Dreapta_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Dreapta_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Stanga_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Stanga_S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Dreapta_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Dreapta_S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Stanga_F.setDirection(DcMotor.Direction.FORWARD);
        Stanga_S.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_F.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_S.setDirection(DcMotor.Direction.REVERSE);

        Brat_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Cutie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Cutie.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Path0", "Starting at %7d :%7d");
                Stanga_S.getCurrentPosition();
                Stanga_F.getCurrentPosition();
                Dreapta_F.getCurrentPosition();
                Dreapta_S.getCurrentPosition();
        telemetry.update();

        //if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        //    init();
       // }
       // else
        //    {
        //    telemetry.addData("Sorry!", "This device is not compatible with TFOD");
       // }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
/*
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (senzorCuloare.red() * SCALE_FACTOR),
                    (int) (senzorCuloare.green() * SCALE_FACTOR),
                    (int) (senzorCuloare.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.

            telemetry.addData("Alpha", senzorCuloare.alpha());
            telemetry.addData("Red  ", senzorCuloare.red());
            telemetry.addData("Green", senzorCuloare.green());
            telemetry.addData("Blue ", senzorCuloare.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run () {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }

        });
*/
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**Inceput:**/

        int i=1;

            straif(1, -6, 0, 0.80);

            Brat_M.setPower(1);
            while(i!=110){
                straif(0,0,0,0);
                i++;
            }
            Brat_M.setPower(0);
            i=1;

            encoderDrive(0.5, 8, -8, 0.45);

            straif(1, -15.5, 0,5);

            Colectare.setPower(1);
            while(i!= 80) {
                straif(0, 0, 0, 0);
                i++;
            }
            i=1;
            Colectare.setPower(0);

            Brat_M.setPower(-1);
            while(i!=110){
                straif(0,0,0,0);
                i++;
            }
            Brat_M.setPower(0);
            i=1;
            straifB(1, 10, 0,6);

            encoderDrive(0.5, -19, 19, 4);

            straif(1, 65, 0,8);


            /*
            if(senzorCuloare.red()==0) {
                straif(0, 0, 0, 0);
            }
            else
                straif(1, 1, 0, 1);
*/

/*            while(i!=160) {
                if (senzorCuloare.red() <= 40) {
                    Stanga_F.setPower(1);
                    Stanga_S.setPower(1);
                    Dreapta_S.setPower(1);
                    Dreapta_F.setPower(1);
                    i++;
                } else {
                    Stanga_F.setPower(0);
                    Stanga_S.setPower(0);
                    Dreapta_S.setPower(0);
                    Dreapta_F.setPower(0);
                    i++;
                }
            }

 */
            i=1;
             telemetry.addData("Path", "Complete");
                telemetry.update();

            }
    public void setSenzorCuloare(){
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        if(opModeIsActive()){

                Color.RGBToHSV((int) (senzorCuloare.red() * SCALE_FACTOR),
                        (int) (senzorCuloare.green() * SCALE_FACTOR),
                        (int) (senzorCuloare.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.

                telemetry.addData("Alpha", senzorCuloare.alpha());
                telemetry.addData("Red  ", senzorCuloare.red());
                telemetry.addData("Green", senzorCuloare.green());
                telemetry.addData("Blue ", senzorCuloare.blue());
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                telemetry.update();
            }
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run () {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }

        });
        }

    public void straif(double speed, double forwardMovement, double lat,double timeoutS) {
        int newLeftTarget_f;
        int newLeftTarget_s;
        int newRightTarget_f;
        int newRightTarget_s;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_f = Stanga_F.getCurrentPosition() + (int) (forwardMovement * COUNTS_PER_INCH);
            newLeftTarget_s = Stanga_S.getCurrentPosition() + (int) (forwardMovement * COUNTS_PER_INCH);
            newRightTarget_f = Dreapta_F.getCurrentPosition() + (int) (forwardMovement * COUNTS_PER_INCH);
            newRightTarget_s = Dreapta_S.getCurrentPosition() + (int) (forwardMovement * COUNTS_PER_INCH);

            int latLeftTarget_f = Stanga_F.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latLeftTarget_s = Stanga_S.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latRightTarget_f = Dreapta_F.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latRightTarget_s = Dreapta_S.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);

            Stanga_F.setTargetPosition(newLeftTarget_f - latLeftTarget_f);
            Stanga_S.setTargetPosition(newLeftTarget_s + latLeftTarget_s);
            Dreapta_F.setTargetPosition(newRightTarget_f - latRightTarget_f);
            Dreapta_S.setTargetPosition(newRightTarget_s + latRightTarget_s);
            // Turn On RUN_TO_POSITION
            Stanga_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Stanga_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            Stanga_F.setPower(Math.abs(speed));
            Stanga_S.setPower(Math.abs(speed));
            Dreapta_F.setPower(Math.abs(speed));
            Dreapta_S.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Stanga_F.isBusy() && Stanga_S.isBusy() && Dreapta_F.isBusy() && Dreapta_S.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d : %7d", newLeftTarget_f, newLeftTarget_s, newRightTarget_s, newRightTarget_f);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d");
                        Stanga_F.getCurrentPosition();
                        Stanga_S.getCurrentPosition();
                        Dreapta_S.getCurrentPosition();
                        Dreapta_F.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            Stanga_F.setPower(0);
            Stanga_S.setPower(0);
            Dreapta_S.setPower(0);
            Dreapta_F.setPower(0);

            Stanga_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Stanga_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void straifB(double speed, double backwardMovement, double lat,double timeoutS) {
        int newLeftTarget_f;
        int newLeftTarget_s;
        int newRightTarget_f;
        int newRightTarget_s;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_f = Stanga_F.getCurrentPosition() + (int) (backwardMovement * COUNTS_PER_INCH);
            newLeftTarget_s = Stanga_S.getCurrentPosition() + (int) (backwardMovement * COUNTS_PER_INCH);
            newRightTarget_f = Dreapta_F.getCurrentPosition() + (int) (backwardMovement * COUNTS_PER_INCH);
            newRightTarget_s = Dreapta_S.getCurrentPosition() + (int) (backwardMovement * COUNTS_PER_INCH);

            int latLeftTarget_f = Stanga_F.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latLeftTarget_s = Stanga_S.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latRightTarget_f = Dreapta_F.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latRightTarget_s = Dreapta_S.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);

            Stanga_F.setDirection(DcMotor.Direction.REVERSE);
            Stanga_S.setDirection(DcMotor.Direction.REVERSE);
            Dreapta_F.setDirection(DcMotor.Direction.FORWARD);
            Dreapta_S.setDirection(DcMotor.Direction.FORWARD);

            Stanga_F.setTargetPosition(newLeftTarget_f - latLeftTarget_f);
            Stanga_S.setTargetPosition(newLeftTarget_s + latLeftTarget_s);
            Dreapta_F.setTargetPosition(newRightTarget_f - latRightTarget_f);
            Dreapta_S.setTargetPosition(newRightTarget_s + latRightTarget_s);
            // Turn On RUN_TO_POSITION
            Stanga_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Stanga_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            Stanga_F.setPower(Math.abs(speed));
            Stanga_S.setPower(Math.abs(speed));
            Dreapta_F.setPower(Math.abs(speed));
            Dreapta_S.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Stanga_F.isBusy() && Stanga_S.isBusy() && Dreapta_F.isBusy() && Dreapta_S.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d : %7d", newLeftTarget_f, newLeftTarget_s, newRightTarget_s, newRightTarget_f);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d");
                Stanga_F.getCurrentPosition();
                Stanga_S.getCurrentPosition();
                Dreapta_S.getCurrentPosition();
                Dreapta_F.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            Stanga_F.setPower(0);
            Stanga_S.setPower(0);
            Dreapta_S.setPower(0);
            Dreapta_F.setPower(0);

            Stanga_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Stanga_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget_f;
        int newLeftTarget_s;
        int newRightTarget_f;
        int newRightTarget_s;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_f = Stanga_F.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTarget_s = Stanga_S.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget_f = Dreapta_F.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarget_s = Dreapta_S.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            Stanga_F.setTargetPosition(newLeftTarget_f);
            Stanga_S.setTargetPosition(newLeftTarget_s);
            Dreapta_F.setTargetPosition(newRightTarget_f);
            Dreapta_S.setTargetPosition(newRightTarget_s);

            // Turn On RUN_TO_POSITION
            Stanga_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Stanga_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            Stanga_F.setPower(Math.abs(speed));
            Stanga_S.setPower(Math.abs(speed));
            Dreapta_F.setPower(Math.abs(speed));
            Dreapta_S.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (Stanga_F.isBusy() && Stanga_S.isBusy() && Dreapta_F.isBusy() && Dreapta_S.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d : %7d", newLeftTarget_f, newLeftTarget_s, newRightTarget_s, newRightTarget_f);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d");
                        Stanga_F.getCurrentPosition();
                        Stanga_S.getCurrentPosition();
                        Dreapta_S.getCurrentPosition();
                        Dreapta_F.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            Stanga_F.setPower(0);
            Stanga_S.setPower(0);
            Dreapta_S.setPower(0);
            Stanga_F.setPower(0);

            Stanga_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Stanga_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Dreapta_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }


}
