package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sushr on 12/6/2017.
 */
@Autonomous(name = "testangle", group = "snake")
public class IteAngle extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();


        goAngle(5,90);
    }
}
