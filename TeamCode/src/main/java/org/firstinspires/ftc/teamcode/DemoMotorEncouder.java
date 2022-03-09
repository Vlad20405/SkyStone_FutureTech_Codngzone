package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


@Autonomous (name="Demo Motor Encouder", group="Kennedy")
@Disabled
public abstract class DemoMotorEncouder extends LinearOpMode {

    HardwareKennedyBot         robot  = new HardwareKennedyBot();
    static final int MOTOR_TICK_COUNT = 1120;

    @Override
     public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*2.938;
        double rotationNeeded = 18/circumference;
        int encouderDrivingTarget = (int)(rotationNeeded*1120);

        robot.leftDrive.setTargetPosition(encouderDrivingTarget);
        robot.rightDrive.setTargetPosition(encouderDrivingTarget);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftDrive.isBusy() ||robot.rightDrive.isBusy()){
            telemetry.addData("Path","Driving 18 inch");
            telemetry.update();
        }

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }




}
