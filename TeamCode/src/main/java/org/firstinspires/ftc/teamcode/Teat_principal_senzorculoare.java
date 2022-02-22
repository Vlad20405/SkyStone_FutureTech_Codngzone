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
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/*
    Codul pentru controlat robotul in TeleOp
 */
@TeleOp(name="Basic: TestCuloare", group="test")
//@Disabled
public class Teat_principal_senzorculoare extends LinearOpMode {

    static final double INCREMENT   = 0.01;
    static final int    CYCLE_MS    =   50;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    // Declaram variabilele
    private ElapsedTime runtime = new ElapsedTime();


    //Motoare brat:
    private DcMotor Brat_M =null;

    private ColorSensor SenzorCuloare;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //cod brat:
        Brat_M= hardwareMap.get(DcMotorImplEx.class, "Brat_M");

        SenzorCuloare = hardwareMap.get(ColorSensor.class, "SenzorCuloare");

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        telemetry.addData(">", "Press Start to begin" );
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            int i=0;



/*
            if(SenzorCuloare.red()>=200 && SenzorCuloare.green()>=200){
                while(i!=100){
                    Brat_M.setPower(1*0.2);
                    i++;
                }
            }
            else
                Brat_M.setPower(gamepad2.left_stick_y);
            i=1;
            telemetry.update();
*/
            Color.RGBToHSV((int) (SenzorCuloare.red() * SCALE_FACTOR),
                    (int) (SenzorCuloare.green() * SCALE_FACTOR),
                    (int) (SenzorCuloare.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", SenzorCuloare.alpha());
            telemetry.addData("Red  ", SenzorCuloare.red());
            telemetry.addData("Green", SenzorCuloare.green());
            telemetry.addData("Blue ", SenzorCuloare.blue());
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

            waitForStart();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }
    }
}