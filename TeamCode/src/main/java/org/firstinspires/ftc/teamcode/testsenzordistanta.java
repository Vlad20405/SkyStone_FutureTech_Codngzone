package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="testsenzordistanta")
@Disabled

public class testsenzordistanta extends LinearOpMode

{

// Declararea OpMode-urilor

    private ElapsedTime runtime = new ElapsedTime();//declararea timpului parcurs
    //declararea motoarelor
    private DcMotor stanga_f=null;
    private DcMotor stanga_s=null;
    private DcMotor dreapta_f=null;
    private DcMotor dreapta_s=null;
    private DistanceSensor sensorRange;
    double power=1; //declararea vitezei

    @Override

    public void runOpMode() throws InterruptedException{

        telemetry.addData("Status", "Initialized");

        telemetry.update();
        //declararea motoarelor pentru a-l putea folosi in program

        stanga_f=hardwareMap.get(DcMotor.class, "Stanga_f");
        stanga_s=hardwareMap.get(DcMotor.class, "Stanga_s");
        dreapta_f=hardwareMap.get(DcMotor.class, "Dreapta_f");
        dreapta_s=hardwareMap.get(DcMotor.class, "Dreapta_s");

        sensorRange = hardwareMap.get(DistanceSensor.class, "Distanta Fata");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange; //Stabilirea Senzorului de distanta REV

        waitForStart();

        runtime.reset();
        // Setarea motoarelor astfel incat sa mearga in fata
        stanga_f.setPower(-power);
        stanga_s.setPower(power);
        dreapta_f.setPower(-power);
        dreapta_s.setPower(power);

        while (opModeIsActive()){

            double p= sensorRange.getDistance(DistanceUnit.CM); // variabila care masoara distanta in cm fata de obstacol

            if(p<30) //Daca distanta fata de obstacol este mai mica de 30 de cm atunci robotul se opreste

            {

                stanga_f.setPower(0);
                stanga_s.setPower(0);
                dreapta_f.setPower(0);
                dreapta_s.setPower(0);

            }

            stanga_f.setPower(power);
            stanga_s.setPower(-power);
            dreapta_f.setPower(power);
            dreapta_s.setPower(-power);
            sleep(1000);// Acesta merge o secunda in spate pentru a evita contactul cu obstacolul

            stanga_f.setPower(0);
            stanga_s.setPower(0);
            dreapta_f.setPower(0);
            dreapta_s.setPower(0);

            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("Status",runtime.toString());
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            telemetry.update();

            idle();//Sfarsit Program

        }

    }

}