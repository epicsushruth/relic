package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

/**
 * Created by Sushr on 12/28/2017.
 */
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
