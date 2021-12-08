package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
    public class Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Variabile motoare:
    private DcMotor Stanga_f = null;
    private DcMotor Dreapta_f = null;
    private DcMotor Stanga_s = null;
    private DcMotor Dreapta_s = null;

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

        //Reset the incoders:
        Stanga_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Stanga_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Dreapta_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Dreapta_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumferinta=3.14*3.7;
        double rotatiaNecesara=18/circumferinta;
        int encouderDrivingTarget= (int) (rotatiaNecesara*1120);

        runtime.reset();
        while (opModeIsActive()) {

            Stanga_f.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            Stanga_s.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            Dreapta_f.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            Dreapta_s.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));

            Stanga_f.setTargetPosition(encouderDrivingTarget);
            Stanga_s.setTargetPosition(encouderDrivingTarget);
            Dreapta_f.setTargetPosition(encouderDrivingTarget);
            Dreapta_s.setTargetPosition(encouderDrivingTarget);

            if(gamepad1.x==true){


                Stanga_f.setPower(.5);
                Stanga_s.setPower(.5);
                Dreapta_f.setPower(.5);
                Dreapta_s.setPower(.5);

                Stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while(Stanga_f.isBusy() || Stanga_s.isBusy() || Dreapta_f.isBusy() || Dreapta_s.isBusy()){

                    telemetry.addData("Path","Driving 18 inch");
                    telemetry.update();
                    idle();
                }
                Stanga_f.setPower(0);
                Stanga_s.setPower(0);
                Dreapta_f.setPower(0);
                Dreapta_s.setPower(0);
            }
            if(gamepad1.y==true){
                Stanga_f.setPower(0);
                Stanga_s.setPower(0);
                Dreapta_f.setPower(0);
                Dreapta_s.setPower(0);
                sleep(500);
            }
            telemetry.addData(">>", "Press start to continue");
            telemetry.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
        }
    }
