package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="manualrobot2")
public class manualrobot2 extends LinearOpMode {

    //@Disabled

    private ElapsedTime runtime=new ElapsedTime();
    private DcMotor stanga_f=null;
    private DcMotor stanga_s=null;
    private DcMotor dreapta_f=null;
    private DcMotor dreapta_s=null;
    private DcMotor brat=null;
    private DcMotor cutie=null;
    private CRServo colectare=null;
    private DcMotor carusel=null;
    DigitalChannel digitalTouch;
    private DistanceSensor sensorRange1;
    private DistanceSensor sensorRange2;
    int contor=0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        dreapta_f = hardwareMap.get(DcMotor.class,"Dreapta_f");
        dreapta_s = hardwareMap.get(DcMotor.class, "Dreapta_s");
        stanga_f = hardwareMap.get(DcMotor.class, "Stanga_f");
        stanga_s = hardwareMap.get(DcMotor.class, "Stanga_s");

        brat=hardwareMap.get(DcMotor.class,"Brat");
        cutie=hardwareMap.get(DcMotor.class,"Cutie");
        colectare=hardwareMap.get(CRServo.class,"Colectare");
        carusel=hardwareMap.get(DcMotor.class,"Carusel");

        digitalTouch=hardwareMap.get(DigitalChannel.class,"digital_touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        sensorRange1=hardwareMap.get(DistanceSensor.class, "sensor_range1");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange1;
        sensorRange2=hardwareMap.get(DistanceSensor.class, "sensor_range2");
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor)sensorRange2;

        stanga_f.setDirection(DcMotor.Direction.FORWARD);
        stanga_s.setDirection(DcMotor.Direction.FORWARD);
        dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Start to begin" );
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            double d1= sensorRange1.getDistance(DistanceUnit.CM); // variabila care masoara distanta in cm fata de obstacol
            double d2= sensorRange2.getDistance(DistanceUnit.CM);

            stanga_f.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
            stanga_s.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            dreapta_f.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            dreapta_s.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));

            brat.setPower(gamepad2.left_stick_y);
            cutie.setPower(gamepad2.right_stick_y*0.2);

            if(gamepad2.x) {
                colectare.setPower(1);
                contor = 2;
            }
            if(gamepad2.b) {
                colectare.setPower(0);
                contor = 1;
            }
            if(gamepad2.y) {
                colectare.setPower(-1);
                contor = 3;
            }

            if(contor==1) {
                telemetry.addData("Colectare","Stop");
            }
            if(contor==2) {
                telemetry.addData("Colectare","ON Forward");
            }
            if(contor==3) {
                telemetry.addData("Colectare","ON Reverse");
            }
            if(gamepad2.left_bumper)
                carusel.setPower(0);
            if(gamepad2.right_bumper)
                carusel.setPower(1);

            telemetry.update();

            if (digitalTouch.getState()==true){
                stanga_f.setPower(0);
                stanga_s.setPower(0);
                dreapta_f.setPower(0);
                dreapta_s.setPower(0);
            }

            if (digitalTouch.getState()==true){
                telemetry.addData("Digital Touch","Is not pressed");
            }
            else {
                telemetry.addData("Digital Touch","Is pressed");
            }
            telemetry.update();

            telemetry.addData("range", String.format("%.01f cm", sensorRange1.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange1.getDistance(DistanceUnit.CM)));
            telemetry.addData("Status",runtime.toString());
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight1.getModelID()));
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight2.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight1.didTimeoutOccur()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight2.didTimeoutOccur()));
            telemetry.update();
        }
    }
}
