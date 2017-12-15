package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Sushr on 12/14/2017.
 */
@Autonomous(name = "proximitySensorTest", group = "fjfj")
public class RevProximitySensorTest extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", bot.distanceSensor.getDistance(DistanceUnit.CM)));


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            telemetry.update();
        }

        // Set the panel back to the default color

    }

    }

