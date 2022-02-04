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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
    Codul pentru controlat robotul in TeleOp
 */
@TeleOp(name="Basic: Linear OpMode", group="test")
//@Disabled
public class Principal2022 extends LinearOpMode {

    static final double INCREMENT   = 0.01;
    static final int    CYCLE_MS    =   50;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    // Declaram variabilele
    private ElapsedTime runtime = new ElapsedTime();

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
    int contor = 0;

    //Senzor distanta:

    //private DistanceSensor sensorRange;

    //Senzor culoare:
  //  private ColorSensor sensorColor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Senzor Distanta Cod:
        //  sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        //cod sasiu:

        Dreapta_F = hardwareMap.get(DcMotor.class, "Dreapta_F");
        Dreapta_S = hardwareMap.get(DcMotor.class, "Dreapta_S");
        Stanga_F = hardwareMap.get(DcMotor.class, "Stanga_F");
        Stanga_S = hardwareMap.get(DcMotor.class, "Stanga_S");


        Stanga_F.setDirection(DcMotor.Direction.FORWARD);
        Stanga_S.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_F.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_S.setDirection(DcMotor.Direction.REVERSE);

        Cutie.setDirection(DcMotor.Direction.REVERSE);
        //cod brat:
        Brat_M= hardwareMap.get(DcMotorImplEx.class, "Brat_M");
        Cutie= hardwareMap.get(DcMotor.class,"Cutie");
        Carusel= hardwareMap.get(DcMotor.class,"Carusel");

        //cod servo:
        Colectare = hardwareMap.get(CRServo.class,"Colectare");
        waitForStart();


        //Variabile Senzor Distanta:
        //telemetry.addData(">>", "Press start to continue");
        //telemetry.update();

        //Variabile senzor culoare:
  /*      sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
*/
        telemetry.addData(">", "Press Start to begin" );
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            //Cod_Miscare_RotiMechanum:
            Stanga_F.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            Stanga_S.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            Dreapta_F.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
            Dreapta_S.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);

            //cod brat:
            Brat_M.setPower(gamepad2.left_stick_y);
            Cutie.setPower(gamepad2.right_stick_y*0.2);

            //cod servo:
            if(gamepad2.x) {
                Colectare.setPower(1);
                contor = 2;
            }
            if(gamepad2.b) {
                Colectare.setPower(0);
                contor = 1;
            }
            if(gamepad2.y) {
                Colectare.setPower(-1);
                contor = 3;
            }

            //Afiseaza modul cutiei de prindere:

            if(contor==1) {
                telemetry.addData("Colectare","Stop");
            }
            if(contor==2) {
                telemetry.addData("Colectare","ON Forward");
            }
            if(contor==3) {
                telemetry.addData("Colectare","ON Reverse");
            }
            //cod motor masă:

            if(gamepad2.a)
                Carusel.setPower(1);
            if(gamepad2.a)
                Carusel.setPower(0);

            //Codare cu senzor culoare:

            //sensorColor.red( );
              //      {
              //      Stanga_S.setPower(0);
              //      Stanga_F.setPower(0);
              //      Dreapta_F.setPower(0);
              //      Dreapta_S.setPower(0);
               //     }
            //Cod_Senzor_Distanta:
            //  telemetry.addData("deviceName",sensorRange.getDeviceName() );
            //    telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));

            //  telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            //  telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

            //Senzor Culoare:
           /* Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });


            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
            */

            //telemetry.addData("Encoder value", Brat_M.getCurrentPosition());
            //telemetry.addData("velocity", Brat_M.getPower());
            //telemetry.addData("position", Brat_M.getCurrentPosition());
            //telemetry.addData("is at target", !Brat_M.isBusy());
            telemetry.update();


            waitForStart();


            telemetry.addData(">>", "Press start to continue");
            telemetry.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }
    }
}