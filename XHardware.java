package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by wolfie on 4/21/17.
 */

public class XHardware {

        DcMotor motorLF;
        DcMotor motorLB;
        DcMotor motorRF;
        DcMotor motorRB;

        HardwareMap hwMap = null;

        public void init(HardwareMap ahwMap) {
            hwMap = ahwMap;

            motorLB = hwMap.dcMotor.get("motorLB");
            motorLF = hwMap.dcMotor.get("motorLF");
            motorRF = hwMap.dcMotor.get("motorRF");
            motorRB = hwMap.dcMotor.get("motorRB");



        }

}
