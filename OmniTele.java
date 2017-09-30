package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by wolfie on 4/28/17.
 */
@TeleOp(name = "Omni TeleOp", group = "Tilerunner")

public class OmniTele extends OpMode{

    OmniWheel robot = new OmniWheel();
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
        robot.motorF.setPower(leftx);
        robot.motorB.setPower(rightx);
        robot.motorL.setPower(lefty);
        robot.motorR.setPower(righty);

    }
}
