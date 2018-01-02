package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sushr on 12/30/2017.
 */
@Autonomous(name = "tytytyhfjnfeeiosdfojhod", group = "fjfjf")
public class driveTest extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        drivingRangeForwardRed();
        drivingRangeBackRed();
        drivingRangeForwardRed();
        stopBotMotors();
    }
}
