package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

/**
 * Created by Sushr on 12/30/2017.
 */
public class driveTest extends Processor {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();

        stopBotMotors();
    }
}
