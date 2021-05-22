package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name= "Test" , group = "FTC")
public class AutonomV2 extends LinearOpMode {

    private DcMotor stanga_f = null;
    private DcMotor dreapta_f = null;
    private DcMotor stanga_s = null;
    private DcMotor dreapta_s = null;

    private DcMotor motor_brat = null;
    private DcMotor motor_brat_colectare = null;
    private DcMotor motor_brat_aruncare=null;

    private Servo servo_aruncare = null;
    private Servo servo_cleste = null;

    @Override
    public void runOpMode() throws InterruptedException{

        stanga_f = hardwareMap.get(DcMotor.class, "stanga_f");
        dreapta_f = hardwareMap.get(DcMotor.class, "dreapta_f");
        stanga_s = hardwareMap.get(DcMotor.class, "stanga_s");
        dreapta_s = hardwareMap.get(DcMotor.class, "dreapta_s");

        motor_brat = hardwareMap.get(DcMotor.class, "motor_brat");
        motor_brat_colectare = hardwareMap.get(DcMotor.class, "motor_brat_colectare");
        motor_brat_aruncare = hardwareMap.get(DcMotor.class,"motor_brat_aruncare");

        servo_aruncare = hardwareMap.get(Servo.class, "servo_aruncare");
        servo_cleste = hardwareMap.get(Servo.class, "servo_cleste");


        stanga_f.setDirection(DcMotor.Direction.FORWARD);
        stanga_s.setDirection(DcMotor.Direction.FORWARD);
        dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            ServoRaise();

            DriveForward(1);
            Stop();
            TotalStop();

            TurnLeft(1);
            TotalStop();

            DriveBackward(1);
            TotalStop();

            ServoLower();

            DriveForward(1);
            TotalStop();

            TurnRight(1);
            TotalStop();

            DriveBackward(2);
            Stop();
            TotalStop();


        }
        idle();
    }
    public void DriveForward (double power){
        stanga_f.setPower(power);
        stanga_s.setPower(power);
        dreapta_f.setPower(power);
        dreapta_s.setPower(power);
    }
    public void  DriveBackward (double power){
        stanga_f.setPower(-power);
        stanga_s.setPower(-power);
        dreapta_f.setPower(-power);
        dreapta_s.setPower(-power);
    }
    public void TurnLeft (double power){
        stanga_f.setPower(-power);
        stanga_s.setPower(-power);
        dreapta_f.setPower(power);
        dreapta_s.setPower(power);
    }
    public void TurnRight (double power){
        stanga_f.setPower(power);
        stanga_s.setPower(power);
        dreapta_f.setPower(-power);
        dreapta_s.setPower(-power);
    }
    public void ServoRaise (){
        servo_cleste.setPosition(1);
    }
    public void ServoLower (){
        servo_cleste.setPosition(0);
    }
    public void Stop(){
        sleep(1000);
    }
    public void TotalStop() {
        sleep(1000);
        DriveForward(0);

    }
}
