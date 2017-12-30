package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sushr on 12/28/2017.
 */
@Autonomous(name = "ultrasonictest",  group = "fjfj")
public class ultraTest extends TestProcessor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            getTelemetry();
        }
    }
}
