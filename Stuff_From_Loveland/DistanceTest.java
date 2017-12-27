package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sushr on 12/23/2017.
 */

@Autonomous(name = "distanceTest", group = "jfjfjf")
public class DistanceTest extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            //telemetry.addData("distance: ", getInchesUltrasonicLeft());
            telemetry.update();
        }
    }
}
