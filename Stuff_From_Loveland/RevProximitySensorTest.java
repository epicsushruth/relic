package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

       }

        // Set the panel back to the default color

    }

    }

