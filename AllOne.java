package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sushr on 10/13/2017.
 */
@Autonomous(name = "Auto one testjfjfrjfjfjfjfj", group ="Concept")

public class AllOne extends TestProcessor{

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {
            bot.motorLB.setPower(1);
            bot.motorLF.setPower(1);
            bot.motorRB.setPower(1);
            bot.motorRF.setPower(1);
        }


    }
}

