package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

/**
 * Created by wolfie on 11/17/17.
 */

public class SCoes extends Processor{
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();





    }
}