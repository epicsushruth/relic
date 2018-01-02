package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/16/2017.
 */
@Autonomous(name = "BluePerpendicular", group = "jfjf")
public class BluePerpendicular extends Processor{
    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        checkVu();
        checkCol();

        grabGlyph();

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        goAngle(20,180);

        align(0);

        turn(90);
        sleep(500);
        align(90);
        sleep(500);
        align(90);
        raiseColorServo();
        drivingRangeForwardBlue();
        drivingRangeBackBlue();
        drivingRangeForwardBlue();
        gotoColumnRight();

        stopBotMotors();


        bot.colorServo.setPosition(0);
        sleep(500);
        align(90);
        //releases the glyph and pushes the glyph into the cryptobox
        score();
        stopBotMotors();
    }
}
