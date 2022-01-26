package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
    public class Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Variabile motoare:
    private DcMotor Stanga_f = null;
    private DcMotor Dreapta_f = null;
    private DcMotor Stanga_s = null;
    private DcMotor Dreapta_s = null;

    //variabile senzori:
    private DistanceSensor senzordistanta1 = null;
    private DistanceSensor senzordistanta2 = null;

    static final int MOTOR_TICK_COUNT=1120;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        Stanga_f= hardwareMap.get(DcMotor.class,"Stanga_f");
        Stanga_s= hardwareMap.get(DcMotor.class,"Stanga_s");
        Dreapta_f= hardwareMap.get(DcMotor.class,"Dreapta_f");
        Dreapta_s=hardwareMap.get(DcMotor.class,"Dreapta_s");

        Stanga_f.setDirection(DcMotor.Direction.FORWARD);
        Stanga_s.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_s.setDirection(DcMotor.Direction.REVERSE);
        //senzori:
        senzordistanta1 = hardwareMap.get(DistanceSensor.class, "senzordistanta1");
        senzordistanta2 = hardwareMap.get(DistanceSensor.class, "senzordistanta2");


        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        while (opModeIsActive()) {

            Stanga_f.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            Stanga_s.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            Dreapta_f.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            Dreapta_s.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));

            //codare senzori:
            telemetry.addData("deviceName",senzordistanta1.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", senzordistanta1.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName",senzordistanta2.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", senzordistanta2.getDistance(DistanceUnit.CM)));
            int i;
            i=0;
            while(i<2) {
                if (senzordistanta1.getDistance(DistanceUnit.CM) <= 2) {
                    i++;
                    Stanga_f.setPower(1);
                    Stanga_s.setPower(1);
                    Dreapta_f.setPower(1);
                    Dreapta_s.setPower(1);
                }

                if (senzordistanta2.getDistance(DistanceUnit.CM) <= 2) {
                    i++;
                    Stanga_f.setPower(-1);
                    Stanga_s.setPower(-1);
                    Dreapta_f.setPower(-1);
                    Dreapta_s.setPower(-1);
                }

            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
        }
    }
