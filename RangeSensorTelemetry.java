package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sushr on 10/28/2017.
 */
@Autonomous(name = "range", group = "fjf")
public class RangeSensorTelemetry extends LinearOpMode {
    TestMap robot = new TestMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
