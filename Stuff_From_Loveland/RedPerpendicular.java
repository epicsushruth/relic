package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/16/2017.
 */
@Autonomous(name = "RedPerpendicular", group = "fjfj")
public class RedPerpendicular extends Processor {

    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        //bot.x = angularOffset();
        checkVu();
        checkCol();
        grabGlyph();

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        goAngle(20, 0);
        sleep(500);
        align(0);
        turn(-90);
        align(-90);
        raiseColorServo();
        drivingRangeForwardRed();
        drivingRangeBackRed();
        drivingRangeForwardRed();
        gotoColumnLeft();

        stopBotMotors();

        bot.colorServo.setPosition(0);
        sleep(500);
        align(-90);
        //releases the glyph and pushes the glyph into the cryptobox
        score();
        stopBotMotors();
    }
}