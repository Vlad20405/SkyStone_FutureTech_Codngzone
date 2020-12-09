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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
    Codul pentru controlat robotul in TeleOp
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

        // Declaram variabilele
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor stanga_f = null;
        private DcMotor dreapta_f = null;
        private DcMotor stanga_s = null;
        private DcMotor dreapta_s = null;
        private DcMotor motor_brat = null;
        private DcMotor motor_cremaliera = null;

        private Servo servo_cleste = null;
        private Servo servo_gimbal_1 = null;
        private Servo servo_gimbal_2 = null;
        private Servo servo_cutie1 = null;
        private Servo servo_cutie2 = null;

        private DistanceSensor sensorRange;

        double pozitie_servo1 = 0;
        double pozitie_servo2 = 0;

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            //sincronizam variabilele cu ce avem in configuratie
            stanga_f = hardwareMap.get(DcMotor.class, "stanga_f");
            dreapta_f = hardwareMap.get(DcMotor.class, "dreapta_f");
            stanga_s = hardwareMap.get(DcMotor.class, "stanga_s");
            dreapta_s = hardwareMap.get(DcMotor.class, "dreapta_s");

            motor_brat = hardwareMap.get(DcMotor.class, "motor_brat");
            motor_cremaliera = hardwareMap.get(DcMotor.class, "motor_cremaliera");

            servo_cleste = hardwareMap.get(Servo.class, "servo_cleste");
            servo_gimbal_1 = hardwareMap.get(Servo.class, "servo_gimba1");
            servo_gimbal_2 = hardwareMap.get(Servo.class, "servo_gimba2");

            servo_cutie1 = hardwareMap.get(Servo.class, "servo_cutie1");
            servo_cutie2 = hardwareMap.get(Servo.class, "servo_cutie2");

            sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
            sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
            //setarile pentru motoare
            stanga_f.setDirection(DcMotor.Direction.FORWARD);
            stanga_s.setDirection(DcMotor.Direction.FORWARD);
            dreapta_f.setDirection(DcMotor.Direction.REVERSE);
            dreapta_s.setDirection(DcMotor.Direction.REVERSE);


            motor_brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_cremaliera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double speedAjust = 7;
            waitForStart();
            runtime.reset();

            servo_gimbal_2.setPosition(0);


            while (opModeIsActive()) {
                if (gamepad1.dpad_down == true) {
                    speedAjust -= 1;
                }
                if (gamepad1.dpad_up == true) {
                    speedAjust += 1;
                }
                if (sensorRange.getDistance(DistanceUnit.CM) <= 7) {
                    stanga_f.setPower(0);
                    stanga_s.setPower(0);
                    dreapta_f.setPower(0);
                    dreapta_s.setPower(0);
                } else {
                //codul pentru mechanum
                stanga_f.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * (+speedAjust / 10));
                stanga_s.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * (+speedAjust / 10));
                dreapta_f.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * (+speedAjust / 10));
                dreapta_s.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * (+speedAjust / 10));

                //codul pentru brat
                motor_brat.setPower(gamepad2.left_stick_y * 0.5);
                motor_cremaliera.setPower(gamepad2.left_trigger);
                motor_cremaliera.setPower(-gamepad2.right_trigger);

                if (gamepad2.x) {
                    pozitie_servo1 = pozitie_servo1 + 0.01;
                }
                if (gamepad2.y) {
                    pozitie_servo1 = pozitie_servo1 - 0.01;
                }
                if (gamepad2.left_bumper) {
                    servo_cleste.setPosition(1);
                }
                if (gamepad2.right_bumper) {
                    servo_cleste.setPosition(0);
                }
                if (gamepad2.dpad_down) {
                    servo_cutie1.setPosition(1);
                    servo_cutie2.setPosition(1);
                }
                if (gamepad2.dpad_up) {

                    servo_cutie1.setPosition(0);
                    servo_cutie2.setPosition(0);

                }
                pozitie_servo1 = Range.clip(pozitie_servo1, 0, 1);
                pozitie_servo2 = Range.clip(pozitie_servo2, 0, 1);
                Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
                telemetry.addData(">>", "Press start to continue");
                telemetry.update();
                servo_gimbal_2.setPosition(pozitie_servo2);
                servo_gimbal_1.setPosition(pozitie_servo1);
                telemetry.addData("speedAjust", speedAjust);
                telemetry.addData("Valoare", gamepad2.right_stick_x);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                    telemetry.update();
                }
            }
        }
    }
