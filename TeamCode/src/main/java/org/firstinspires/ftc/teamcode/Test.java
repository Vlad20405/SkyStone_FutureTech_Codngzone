package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
    public class Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Variabile motoare:
    private DcMotor Stanga_f = null;
    private DcMotor Dreapta_f = null;
    private DcMotor Stanga_s = null;
    private DcMotor Dreapta_s = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Stanga_f= hardwareMap.get(DcMotor.class,"Stanga_f");
        Stanga_s= hardwareMap.get(DcMotor.class,"Stanga_s");
        Dreapta_f= hardwareMap.get(DcMotor.class,"Dreapta_f");
        Dreapta_s=hardwareMap.get(DcMotor.class,"Dreapta_s");

        Stanga_f.setDirection(DcMotor.Direction.FORWARD);
        Stanga_s.setDirection(DcMotor.Direction.FORWARD);
        Dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        Dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            Stanga_f.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            Stanga_s.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            Dreapta_f.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            Dreapta_s.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));

            telemetry.addData(">>", "Press start to continue");
            telemetry.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
        }
    }
