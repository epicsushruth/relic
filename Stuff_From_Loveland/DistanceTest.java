package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

/**
 * Created by Sushr on 12/23/2017.
 */

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
