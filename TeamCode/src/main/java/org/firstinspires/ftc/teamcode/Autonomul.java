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



@Autonomous(name="Autonomul_bun", group="Pushbot")
//@Disabled
public class Autonomul extends LinearOpMode {

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor stanga_fata = null;
    private DcMotor dreapta_fata = null;
    private DcMotor stanga_spate = null;
    private DcMotor dreapta_spate = null;
    private DcMotor motor_brat= null;
    private DcMotor motor_cremaliera = null;

    @Override
    public void runOpMode() {


        stanga_fata  = hardwareMap.get(DcMotor.class, "stanga_f");
        dreapta_fata = hardwareMap.get(DcMotor.class, "dreapta_f");
        stanga_spate = hardwareMap.get(DcMotor.class, "stanga_s");
        dreapta_spate = hardwareMap.get(DcMotor.class, "dreapta_s");

        motor_brat=hardwareMap.get(DcMotor.class,"motor_brat");
        motor_cremaliera=hardwareMap.get(DcMotor.class,"motor_cremaliera");

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        stanga_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stanga_spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga_fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreapta_fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreapta_spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stanga_spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0",  "Starting at %7d :%7d :7d :7d",
                             stanga_fata.getCurrentPosition(),
                            dreapta_fata.getCurrentPosition(),
                            stanga_spate.getCurrentPosition(),
                            dreapta_spate.getCurrentPosition());
        telemetry.update();


        waitForStart();

        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);
        encoderDrive(TURN_SPEED,   12, -12, 4.0);
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {





        }
    }
}
