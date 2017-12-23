package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sushr on 12/21/2017.
 */
@Autonomous(name = "distance", group = "fjfj")
public class testDistance extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("RangeSensor(CM): ",bot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("ProximitySensor(CM)", bot.colorSensor2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
