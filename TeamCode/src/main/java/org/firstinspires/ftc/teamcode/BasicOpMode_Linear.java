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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class BasicOpMode_Linear extends LinearOpMode {

    // Declaram variabilele
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor stanga_fata = null;
    private DcMotor dreapta_fata = null;
    private DcMotor stanga_spate = null;
    private DcMotor dreapta_spate = null;
    private DcMotor motor_brat= null;
    private DcMotor motor_cremaliera = null;

    private Servo servo_cleste=null;
    private Servo servo_gimbal_1=null;
    private Servo servo_gimbal_2=null;
    private Servo servo_cutie=null;

    double pozitie_servo1=0;
    double pozitie_servo2=0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //sincronizam variabilele cu ce avem in configuratie
        stanga_fata  = hardwareMap.get(DcMotor.class, "stanga_f");
        dreapta_fata = hardwareMap.get(DcMotor.class, "dreapta_f");
        stanga_spate = hardwareMap.get(DcMotor.class, "stanga_s");
        dreapta_spate = hardwareMap.get(DcMotor.class, "dreapta_s");

        motor_brat=hardwareMap.get(DcMotor.class,"motor_brat");
        motor_cremaliera=hardwareMap.get(DcMotor.class,"motor_cremaliera");

        servo_cleste=hardwareMap.get(Servo.class,"servo_cleste");
        servo_gimbal_1=hardwareMap.get(Servo.class,"servo_gimba1");
        servo_gimbal_2=hardwareMap.get(Servo.class,"servo_gimba2");


        //setarile pentru motoare
        stanga_fata.setDirection(DcMotor.Direction.FORWARD);
        stanga_spate.setDirection(DcMotor.Direction.FORWARD);
        dreapta_fata.setDirection(DcMotor.Direction.REVERSE);
        dreapta_spate.setDirection(DcMotor.Direction.REVERSE);

        motor_brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_cremaliera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        servo_gimbal_2.setPosition(0);


        while (opModeIsActive()) {

            //codul pentru mechanum
            stanga_fata.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x);
            stanga_spate.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x);
            dreapta_fata.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x);
            dreapta_spate.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x);


            //codul pentru brat
            motor_brat.setPower(gamepad2.left_stick_y*0.5);
            motor_cremaliera.setPower(gamepad2.left_trigger);
            motor_cremaliera.setPower(-gamepad2.right_trigger);

            if(gamepad2.a){
                pozitie_servo2=pozitie_servo2+0.01;
            }
            if (gamepad2.b){
                pozitie_servo2=pozitie_servo2-0.01;
            }
            if(gamepad2.x){
                pozitie_servo1=pozitie_servo1+0.01;
            }
            if(gamepad2.y){
                pozitie_servo1=pozitie_servo1-0.01;
            }
            if(gamepad2.left_bumper) {
                servo_cleste.setPosition(1);
            }
            if(gamepad2.right_bumper) {
                servo_cleste.setPosition(0);
            }
            if(gamepad2.dpad_down){
                servo_cutie.setPosition(1);
            }
            if(gamepad2.dpad_up){

                servo_cutie.setPosition(0);
            }
            pozitie_servo1=Range.clip(pozitie_servo1,0,1);
            pozitie_servo2=Range.clip(pozitie_servo2,0,1);

            servo_gimbal_2.setPosition(pozitie_servo2);
            servo_gimbal_1.setPosition(pozitie_servo1);

            telemetry.addData("Valoare",gamepad2.right_stick_x);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}