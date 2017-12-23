package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/15/2017.
 */
@Autonomous(name = "redParallel", group = "fjfrjkdk")
public class RedParallel extends Processor{

    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        checkVu();
        checkCol();
        bot.glyphServo3.setPosition(.35);
        bot.glyphServo4.setPosition(.5);

        sleep(1500);

        runtime.reset();

        //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(700);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        goAngle(20,0);
        sleep(1000);
        align(0);
        sleep(1000);
        turn(-180);
        sleep(2000);
        align(180);
        sleep(1000);
        raiseColorServo();
        drivingRangeForward();
        drivingRangeBack();


        gotoColumnLeft();


        sleep(1000);
        bot.colorServo.setPosition(0);
        sleep(1000);
        align(180);
        driveToDistance();
        sleep(1000);
        align(180);
        score();
        stopBotMotors();
    }
}
