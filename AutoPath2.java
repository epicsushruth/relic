package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sushr on 10/13/2017.
 */
@Autonomous(name = "Auto 3xr", group ="Concept")

public class AutoPath2 extends TestProcessor{

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();

        strafeRight(10,.4);
    }
}

