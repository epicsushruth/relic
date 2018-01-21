package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/15/2017.
 */
@Autonomous(name = "2glyphRedParallel", group = "fjfrjkdk")
public class TwoglyphRedParallel extends Processor {

    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        checkCol();
        grabGlyph();
        align(0);



        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        goAngle(22.5,20);

        align(0);
        turn(-180);
        align(180);
        raiseColorServo();
        align(180);

        adjust(.036);
        adjust(.036);
        adjust(.036);
        goAngle(2,0);

        align(180);

        sleep(500);
        gotoColumnLeft();

        bot.colorServo.setPosition(0);

        sleep(500);
        align(180);
        //driveToDistance();
        score1(180);
        stopBotMotors();
        turn(180);
        goAngle(31.8,90);
        openLeftBot();
        openRightBot();
        goAngle(2,90);
        gripGlyphBot();
        goAngle(10,270);
        align(0);
        turn(180);
        goAngle(35,90);
    }
}