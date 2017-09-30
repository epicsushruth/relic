package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Sushr on 6/28/2017.
 */
@Autonomous(name = "omnitest")
public class omnitest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OmniWheel robot = new OmniWheel();
        waitForStart();
        robot.motorF.setPower(1);
        robot.motorB.setPower(-1);
        robot.motorL.setPower(1);
        robot.motorR.setPower(-1);
    }
}
