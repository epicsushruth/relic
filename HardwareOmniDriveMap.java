package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Sushr on 9/1/2017.
 */

public class HardwareOmniDriveMap{
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorFront = null;
    public DcMotor motorBack = null;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight = hwMap.dcMotor.get("motorRight");
        motorFront = hwMap.dcMotor.get("motorFront");
        motorBack = hwMap.dcMotor.get("motorBack");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorBack.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }
}
