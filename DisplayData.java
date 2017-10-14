package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by wolfie on 9/1/17.
 */
@Autonomous(name = "je", group = "jij")
public class DisplayData extends LinearOpMode{
    Map robot = new Map();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            if(robot.imu.isSystemCalibrated()){
                telemetry.addData("calibrated","system");

            }
            if(robot.imu.isGyroCalibrated()){
                telemetry.addData("calabrated", "gyro");
            }
            telemetry.addData("gyro %f", robot.imu.getAngularOrientation());
            telemetry.update();
        }
    }
}
