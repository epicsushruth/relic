package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

/**
 * Created by Sushr on 12/6/2017.
 */
public class IteAngle extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();


        goAngle(5,90);
    }
}
