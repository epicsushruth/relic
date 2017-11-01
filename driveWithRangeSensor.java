package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sushr on 10/28/2017.
 */
@Autonomous(name = "driveWithRangeSensor", group = "RangeSensor")
public class driveWithRangeSensor extends TestProcessor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        checkCol();

        while(bot.rangeSensor.getDistance(DistanceUnit.CM)>25)
        {
            bot.motorLB.setPower(0.2);
            bot.motorRB.setPower(-0.2);
            bot.motorRF.setPower(0.2);
            bot.motorLF.setPower(-0.2);
        }
    }
}
