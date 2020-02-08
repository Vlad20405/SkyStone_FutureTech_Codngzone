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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

@Autonomous(name="Pushbot: Auto By Encoder", group="Pushbot")
//@Disabled
public class

adsfasdfasdfas extends LinearOpMode {

    /* Declare OpMode members. */


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AStJnD3/////AAABmSbeJsgIn0i0lCVUWxTrdiJtlmoJSTgwubOWegTxmEdsFzJkBuin+7gNMqUApOj8XkwLsbgKdDM2VsiC5ttIAUxBasPjSgQ5NLOLbkEX8E5hSWmmO73F3SBXRKP43WSNSCYDNRQdC3ZuGaVNn/3Xt3K5P82/890LHtbxK1NJc+R8bGZEH2bCAly6e1xYGkTbfaGSHkvnIxtQQl3XstJL9Q96D9MSZEcbSjr8JYB5NsoOujufJRkxsIhtmpshfxzyHNs9Xo+4QlQL2AHj/F0NCYMfqOTk19C12o8jJ2YeQkHEib2OBHmVKMi+V/ptEAEeRTmkEHBNNew8j5sd0gNLJmTo/CXFy3f/Fp0ZBgM21dy2";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor stanga_f=null;
    private  DcMotor stanga_s=null;
    private  DcMotor dreapta_f=null;
    private  DcMotor dreapta_s=null;

    private DcMotor motor_brat= null;
    private DcMotor motor_cremaliera = null;

    private Servo servo_cleste=null;
    private Servo servo_gimbal_1=null;
    private Servo servo_gimbal_2=null;
    private Servo servo_cutie=null;
    

    @Override
    public void runOpMode() {




        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        stanga_f  = hardwareMap.get(DcMotor.class, "stanga_f");
        stanga_s = hardwareMap.get(DcMotor.class, "stanga_s");
        dreapta_f = hardwareMap.get(DcMotor.class,"dreapta_f");
        dreapta_s = hardwareMap.get(DcMotor.class,"dreapta_s");

        motor_brat=hardwareMap.get(DcMotor.class,"motor_brat");
        motor_cremaliera=hardwareMap.get(DcMotor.class,"motor_cremaliera");

        servo_cleste=hardwareMap.get(Servo.class,"servo_cleste");
        servo_gimbal_1=hardwareMap.get(Servo.class,"servo_gimba1");
        servo_gimbal_2=hardwareMap.get(Servo.class,"servo_gimba2");

        servo_cutie=hardwareMap.get(Servo.class,"servo_cutie");

        stanga_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stanga_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stanga_s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreapta_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreapta_s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stanga_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stanga_s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreapta_s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stanga_f.setDirection(DcMotor.Direction.FORWARD);
        stanga_s.setDirection(DcMotor.Direction.FORWARD);
        dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        motor_cremaliera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          stanga_f.getCurrentPosition(),
                          stanga_s.getCurrentPosition());
        telemetry.update();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        brat(0.3,-30,10);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void straif(double speed,
                             double forwardMovement, double lat,
                             double timeoutS) {
        int newLeftTarget_f;
        int newLeftTarget_s;
        int newRightTarget_f;
        int newRightTarget_s;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_f = stanga_f.getCurrentPosition() + (int)(forwardMovement * COUNTS_PER_INCH);
            newLeftTarget_s = stanga_s.getCurrentPosition() + (int)(forwardMovement * COUNTS_PER_INCH);
            newRightTarget_f= dreapta_f.getCurrentPosition() + (int) (forwardMovement * COUNTS_PER_INCH);
            newRightTarget_s= dreapta_s.getCurrentPosition() + (int) (forwardMovement *COUNTS_PER_INCH);

            int latLeftTarget_f = stanga_f.getCurrentPosition() + (int)( lat* COUNTS_PER_INCH);
            int latLeftTarget_s = stanga_s.getCurrentPosition() + (int)(lat * COUNTS_PER_INCH);
            int latRightTarget_f= dreapta_f.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);
            int latRightTarget_s = dreapta_s.getCurrentPosition() + (int) (lat * COUNTS_PER_INCH);

            stanga_f.setTargetPosition(newLeftTarget_f-latLeftTarget_f);
            stanga_s.setTargetPosition(newLeftTarget_s+latLeftTarget_s);
            dreapta_f.setTargetPosition(newRightTarget_f-latRightTarget_f);
            dreapta_s.setTargetPosition(newRightTarget_s+latRightTarget_s);
            // Turn On RUN_TO_POSITION
            stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            stanga_f.setPower(Math.abs(speed));
            stanga_s.setPower(Math.abs(speed));
            dreapta_f.setPower(Math.abs(speed));
            dreapta_s.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (stanga_f.isBusy() && stanga_s.isBusy() &&dreapta_f.isBusy() && dreapta_s.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d : %7d : %7d" , newLeftTarget_f,  newLeftTarget_s, newRightTarget_s, newRightTarget_f);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                                            stanga_f.getCurrentPosition(),
                                            stanga_s.getCurrentPosition(), dreapta_s.getCurrentPosition(), dreapta_f.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stanga_f.setPower(0);
            stanga_s.setPower(0);
            dreapta_s.setPower(0);
            dreapta_f.setPower(0);

            stanga_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stanga_s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget_f;
        int newLeftTarget_s;
        int newRightTarget_f;
        int newRightTarget_s;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget_f = stanga_f.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget_s = stanga_s.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget_f= dreapta_f.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarget_s= dreapta_s.getCurrentPosition() + (int) (rightInches *COUNTS_PER_INCH);
            stanga_f.setTargetPosition(newLeftTarget_f);
            stanga_s.setTargetPosition(newLeftTarget_s);
            dreapta_f.setTargetPosition(newRightTarget_f);
            dreapta_s.setTargetPosition(newRightTarget_s);

            // Turn On RUN_TO_POSITION
            stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            stanga_f.setPower(Math.abs(speed));
            stanga_s.setPower(Math.abs(speed));
            dreapta_f.setPower(Math.abs(speed));
            dreapta_s.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (stanga_f.isBusy() && stanga_s.isBusy() &&dreapta_f.isBusy() && dreapta_s.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d : %7d : %7d" , newLeftTarget_f,  newLeftTarget_s, newRightTarget_s, newRightTarget_f);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        stanga_f.getCurrentPosition(),
                        stanga_s.getCurrentPosition(), dreapta_s.getCurrentPosition(), dreapta_f.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stanga_f.setPower(0);
            stanga_s.setPower(0);
            dreapta_s.setPower(0);
            stanga_f.setPower(0);

            stanga_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stanga_s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void cremaliera(double speed, double movement, double timeoutS) {
        int movement_do;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            movement_do = motor_cremaliera.getCurrentPosition() + (int)(movement * COUNTS_PER_INCH);


            motor_cremaliera.setTargetPosition(movement_do);


            // Turn On RUN_TO_POSITION
            motor_cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor_cremaliera.setPower(Math.abs(speed));



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor_cremaliera.isBusy() )) {

                telemetry.addData("Path1" , movement_do);
                telemetry.addData("Path2", motor_cremaliera.getCurrentPosition());
            }

            // Stop all motion;
            motor_cremaliera.setPower(0);

            motor_cremaliera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void brat(double speed, double movement, double timeoutS) {
        int movement_do;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            movement_do = motor_brat.getCurrentPosition() + (int)(movement * COUNTS_PER_INCH);


            motor_brat.setTargetPosition(movement_do);


            // Turn On RUN_TO_POSITION
            motor_brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor_brat.setPower(Math.abs(speed));



            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor_brat.isBusy())) {

                telemetry.addData("Path1" , movement_do);
                telemetry.addData("Path2", motor_cremaliera.getCurrentPosition());
            }

            // Stop all motion;
            motor_brat.setPower(0);

            motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
