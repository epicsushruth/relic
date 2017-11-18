package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.control.Omni;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;

/**
 * Manual demo for FTC Relic Recovery game.
 */
@TeleOp(name="Omni.Manual", group="pmtischler")
public class RelicRecoveryManual extends RobotHardware {

    @Override
    public void loop() {
        setDriveForOmniForSpeed(Omni.joystickToMotion(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y));

        if (gamepad1.dpad_up) {
            raiseJewelArm();
        } else if (gamepad1.dpad_down) {
            lowerJewelArm();
        }

        if (gamepad1.dpad_right) {
            forwardJewelArm();
        } else if (gamepad1.dpad_left) {
            backwardJewelArm();
        } else {
            centerJewelArm();
        }
    }
}

