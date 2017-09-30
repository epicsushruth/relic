package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by wolfie on 4/28/17.
 */

public class OmniWheel {
    DcMotor motorF;
    DcMotor motorB;
    DcMotor motorR;
    DcMotor motorL;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        motorL = hwMap.dcMotor.get("motorL");
        motorR = hwMap.dcMotor.get("motorR");
        motorF = hwMap.dcMotor.get("motorF");
        motorB = hwMap.dcMotor.get("motorB");



    }

}
