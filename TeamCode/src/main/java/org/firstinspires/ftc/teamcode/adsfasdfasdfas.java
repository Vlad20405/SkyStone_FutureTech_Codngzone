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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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
public class adsfasdfasdfas extends LinearOpMode {

    /* Declare OpMode members. */
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
    

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        stanga_f  = hardwareMap.get(DcMotor.class, "stanga_f");
        stanga_s = hardwareMap.get(DcMotor.class, "stanga_s");
        dreapta_f = hardwareMap.get(DcMotor.class,"dreapta_f");
        dreapta_s = hardwareMap.get(DcMotor.class,"dreapta_s");

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

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          stanga_f.getCurrentPosition(),
                          stanga_s.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout



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

            stanga_f.setTargetPosition(newLeftTarget_f+latLeftTarget_f);
            stanga_s.setTargetPosition(newLeftTarget_s-latLeftTarget_s);
            dreapta_f.setTargetPosition(newRightTarget_f-latRightTarget_f);
            dreapta_s.setTargetPosition(newRightTarget_s+latRightTarget_f);
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

            stanga_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stanga_s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
