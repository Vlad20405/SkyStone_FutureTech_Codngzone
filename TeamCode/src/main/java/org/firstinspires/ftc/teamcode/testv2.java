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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
    Codul pentru controlat robotul in TeleOp
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class testv2 extends LinearOpMode {

    // Declaram variabilele
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Dreapta_F = null;
    private DcMotor Dreapta_S = null;
    private DcMotor Stanga_F = null;
    private DcMotor Stanga_S = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //sincronizam variabilele cu ce avem in configuratie

        Dreapta_F= hardwareMap.get(DcMotor.class,"Dreapta_F");
        Dreapta_S= hardwareMap.get(DcMotor.class,"Dreapta_S");
        Stanga_F= hardwareMap.get(DcMotor.class,"Stanga_F");
        Stanga_S= hardwareMap.get(DcMotor.class,"Stanga_S");

        Stanga_F.setDirection(DcMotor.Direction.FORWARD);
        Stanga_S.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_F.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_S.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            Stanga_F.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            Stanga_S.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) ;
            Dreapta_F.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) ;
            Dreapta_S.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);



            waitForStart();


            telemetry.addData(">>", "Press start to continue");
            telemetry.update();

            //servo_gimbal_1.setPosition(pozitie_servo1);

            telemetry.addData("Valoare", gamepad2.right_stick_x);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }
    }

}