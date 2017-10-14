package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by wolfie on 9/15/17.
 */
@TeleOp(name = "XTele", group = "X")
public class CougarTeleop extends OpMode{
    CougarMap robot = new CougarMap();
    double lefty;
    double righty;
    double leftx;
    double rightx;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        leftx = gamepad1.left_stick_x;
        lefty = gamepad1.left_stick_y;
        rightx = gamepad1.right_stick_x;
        righty = gamepad1.right_stick_y;
        robot.motorLB.setPower(lefty);
        robot.motorLF.setPower(lefty);
        robot.motorRB.setPower(righty);
        robot.motorRF.setPower(righty);

    }
}
