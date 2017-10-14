package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sushr on 9/23/2017.
 */

public class VuforiaCoordinateSystemHardwareMap {
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorFront = null;
    public DcMotor motorBack = null;





    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft = hwMap.dcMotor.get("motor left");
        motorRight = hwMap.dcMotor.get("motor right");
        motorFront = hwMap.dcMotor.get("motor front");
        motorBack = hwMap.dcMotor.get("motor back");

        //colorSensor = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("colorSensor");

        //flickServo = hwMap.servo.get("flickServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorFront.setDirection(DcMotor.Direction.FORWARD);
        motorBack.setDirection(DcMotor.Direction.REVERSE);

    }
}