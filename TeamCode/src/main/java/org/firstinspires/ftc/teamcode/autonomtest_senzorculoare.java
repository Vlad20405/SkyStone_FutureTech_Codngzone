package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import android.graphics.Color;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "testtt")
//@Disabled

public class autonomtest_senzorculoare extends LinearOpMode {

    ColorSensor sensorColor;

    DistanceSensor sensorDistance;
    // declararea motoarelor
    private DcMotor Stanga_f = null;
    private DcMotor Stanga_s = null;
    private DcMotor Dreapta_f = null;
    private DcMotor Dreapta_s = null;
    int contor = 0;

    @Override

    public void runOpMode() {

        sensorColor = hardwareMap.get(ColorSensor.class, "senzorCuloare");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "senzorDistanta");

        Stanga_f = hardwareMap.get(DcMotor.class, "Stanga_f");
        Stanga_s = hardwareMap.get(DcMotor.class, "Stanga_s");
        Dreapta_f = hardwareMap.get(DcMotor.class, "Dreapta_f");
        Dreapta_s = hardwareMap.get(DcMotor.class, "Dreapta_s");

        Stanga_f.setDirection(DcMotor.Direction.FORWARD);
        Stanga_s.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        sensorColor.enableLed(false);

        sensorColor.enableLed(true);

        Stanga_f.setPower(1);
        Stanga_s.setPower(1);
        Dreapta_f.setPower(1);
        Dreapta_s.setPower(1);

        while (sensorColor.red() < 150 && contor == 0) {
            contor++;
            Stanga_f.setPower(0.5);
            Stanga_s.setPower(0.5);
            Dreapta_f.setPower(0.5);
            Dreapta_s.setPower(0.5);
        }
        while (sensorColor.red() < 150 && contor == 1) {
            Stanga_f.setPower(0);
            Stanga_s.setPower(0);
            Dreapta_f.setPower(0);
            Dreapta_s.setPower(0);
        }

        Stanga_f.setPower(0);
        Stanga_s.setPower(0);
        Dreapta_f.setPower(0);
        Dreapta_s.setPower(0);

        while (opModeIsActive()) {

            telemetry.addData("ARGB", sensorColor.argb());

            telemetry.update();

        }

    }
}
