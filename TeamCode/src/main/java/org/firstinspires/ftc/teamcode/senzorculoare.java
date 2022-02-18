
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;


@TeleOp(name = "Sensor: REVColorDistance", group = "Sensor")
@Disabled
public class senzorculoare extends LinearOpMode {


     ColorSensor sensorColor;
     DistanceSensor sensorDistance;

     @Override
     public void runOpMode() {
          sensorColor = hardwareMap.get(ColorSensor.class, "senzor_culoare");
          sensorDistance = hardwareMap.get(DistanceSensor.class, "senzor_distanta");
          float hsvValues[] = {0F, 0F, 0F};
          final float values[] = hsvValues;

          final double SCALE_FACTOR = 255;


          int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
          final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

          waitForStart();

          while (opModeIsActive()) {

               Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                       (int) (sensorColor.green() * SCALE_FACTOR),
                       (int) (sensorColor.blue() * SCALE_FACTOR),
                       hsvValues);

               telemetry.addData("Distanta (cm)",
                       String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
               telemetry.addData("Alfa", sensorColor.alpha());
               telemetry.addData("Rosu", sensorColor.red());
               telemetry.addData("Verde", sensorColor.green());
               telemetry.addData("Albastru", sensorColor.blue());
               telemetry.addData("Nuanta", hsvValues[0]);

               relativeLayout.post(new Runnable() {
                    public void run() {
                         relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
               });

               telemetry.update();
          }
          relativeLayout.post(new Runnable() {
               public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
               }
          });
     }
}