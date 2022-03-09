package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto")
//@Disabled
public class Auto extends LinearOpMode {
    private DcMotor stanga_f=null;
    private DcMotor stanga_s=null;
    private DcMotor dreapta_f=null;
    private DcMotor dreapta_s=null;
    private int stanga_fPoz=0;
    private int stanga_sPoz=0;
    private int dreapta_fPoz=0;
    private int dreapta_sPoz=0;
    private DcMotor carusel=null;
    private DcMotor brat_m=null;
    private DcMotor cutie=null;
    private DcMotor colectare=null;
    DigitalChannel digitalTouch;
    double speed=1;
    double power=1;
    //private ElapsedTime runtime = new ElapsedTime();//declararea timpului parcurs
    @Override
    public void runOpMode(){
        stanga_f=hardwareMap.get(DcMotor.class, "Stanga_F");
        stanga_s=hardwareMap.get(DcMotor.class,"Stanga_S");
        dreapta_f=hardwareMap.get(DcMotor.class,"Dreapta_F");
        dreapta_s=hardwareMap.get(DcMotor.class,"Dreapta_S");
        carusel=hardwareMap.get(DcMotor.class,"Carusel");
        brat_m=hardwareMap.get(DcMotor.class,"Brat_M");
        cutie=hardwareMap.get(DcMotor.class,"Cutie");

        digitalTouch=hardwareMap.get(DigitalChannel.class,"digital_touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        stanga_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stanga_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreapta_s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stanga_f.setDirection(DcMotor.Direction.FORWARD);
        stanga_s.setDirection(DcMotor.Direction.FORWARD);
        dreapta_f.setDirection(DcMotor.Direction.REVERSE);
        dreapta_s.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        drive(2000, 2000, 2000, 2000, 1);//inainte
        drive(-1400, -1400, 1400, 1400, 1);//rotire stanga
        drive(7000, 7000, 7000, 7000, 1);//inainte
        drive(-1500, -1500, 1500, 1500, 1);//rotire stanga
        drive(2350, 2350, 2350, 2350, 1);//inainte
        if (digitalTouch.getState()==false){
            speed=0;
        }
        carusel.setPower(1);
        sleep(4500);//this.sleep();
        carusel.setPower(0);
        drive(-1700,-1700,-1700,-1700,1);//inapoi
        drive(-1600,-1600,1600,1600,1);//rotire stanga
        drive(13000,13000,13000,13000,1);//inainte

    }
    private void drive(int stanga_fTarget, int stanga_sTarget, int dreapta_fTarget, int dreapta_sTarget, double speed) {
        stanga_fPoz += stanga_fTarget;
        stanga_sPoz += stanga_sTarget;
        dreapta_fPoz += dreapta_fTarget;
        dreapta_sPoz += dreapta_sTarget;

        stanga_f.setTargetPosition(stanga_fPoz);
        stanga_s.setTargetPosition(stanga_sPoz);
        dreapta_f.setTargetPosition(dreapta_fPoz);
        dreapta_s.setTargetPosition(dreapta_sPoz);

        stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga_f.setPower(-speed);
        stanga_s.setPower(-speed);
        dreapta_f.setPower(-speed);
        dreapta_s.setPower(-speed);

        while (opModeIsActive()&&stanga_f.isBusy()&&stanga_s.isBusy()&&dreapta_f.isBusy()&&dreapta_s.isBusy()){
            if (digitalTouch.getState()==true){
                telemetry.addData("Digital Touch","Is not pressed");
            }
            else {
                telemetry.addData("Digital Touch","Is pressed");
            }
            telemetry.update();
            idle();
        }
    }
    private void driveForward(int forwardTarget,double speed){
        stanga_fPoz += forwardTarget;
        stanga_sPoz += forwardTarget;
        dreapta_fPoz += forwardTarget;
        dreapta_sPoz += forwardTarget;

        stanga_f.setTargetPosition(stanga_fPoz);
        stanga_s.setTargetPosition(stanga_sPoz);
        dreapta_f.setTargetPosition(dreapta_fPoz);
        dreapta_s.setTargetPosition(dreapta_sPoz);

        stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga_f.setPower(-speed);
        stanga_s.setPower(-speed);
        dreapta_f.setPower(-speed);
        dreapta_s.setPower(-speed);

        while (opModeIsActive()&&stanga_f.isBusy()&&stanga_s.isBusy()&&dreapta_f.isBusy()&&dreapta_s.isBusy()){
            idle();
        }
    }
    private void driveBackward(int backwardTarget,double speed){
        stanga_fPoz += -backwardTarget;
        stanga_sPoz += -backwardTarget;
        dreapta_fPoz += -backwardTarget;
        dreapta_sPoz += -backwardTarget;

        stanga_f.setTargetPosition(stanga_fPoz);
        stanga_s.setTargetPosition(stanga_sPoz);
        dreapta_f.setTargetPosition(dreapta_fPoz);
        dreapta_s.setTargetPosition(dreapta_sPoz);

        stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga_f.setPower(-speed);
        stanga_s.setPower(-speed);
        dreapta_f.setPower(-speed);
        dreapta_s.setPower(-speed);

        while (opModeIsActive()&&stanga_f.isBusy()&&stanga_s.isBusy()&&dreapta_f.isBusy()&&dreapta_s.isBusy()){
            idle();
        }
    }
    private void turnLeft(int leftTarget, double speed){
        stanga_fPoz += -leftTarget;
        stanga_sPoz += -leftTarget;
        dreapta_fPoz += leftTarget;
        dreapta_sPoz += leftTarget;

        stanga_f.setTargetPosition(stanga_fPoz);
        stanga_s.setTargetPosition(stanga_sPoz);
        dreapta_f.setTargetPosition(dreapta_fPoz);
        dreapta_s.setTargetPosition(dreapta_sPoz);

        stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga_f.setPower(-speed);
        stanga_s.setPower(-speed);
        dreapta_f.setPower(-speed);
        dreapta_s.setPower(-speed);

        while (opModeIsActive()&&stanga_f.isBusy()&&stanga_s.isBusy()&&dreapta_f.isBusy()&&dreapta_s.isBusy()){
            idle();
        }
    }
    private void turnRight(int rightTarget,double speed){
        stanga_fPoz += rightTarget;
        stanga_sPoz += rightTarget;
        dreapta_fPoz += -rightTarget;
        dreapta_sPoz += -rightTarget;

        stanga_f.setTargetPosition(stanga_fPoz);
        stanga_s.setTargetPosition(stanga_sPoz);
        dreapta_f.setTargetPosition(dreapta_fPoz);
        dreapta_s.setTargetPosition(dreapta_sPoz);

        stanga_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stanga_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreapta_s.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        stanga_f.setPower(-speed);
        stanga_s.setPower(-speed);
        dreapta_f.setPower(-speed);
        dreapta_s.setPower(-speed);

        while (opModeIsActive()&&stanga_f.isBusy()&&stanga_s.isBusy()&&dreapta_f.isBusy()&&dreapta_s.isBusy()){
            idle();
        }
    }

}
