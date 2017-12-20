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
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        bot.glyphServo3.setPosition(.35);
        bot.glyphServo4.setPosition(.5);
        sleep(1000);

        runtime.reset();

        //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        goAngle(10,0);
        align(0);
        turn(180);
        align(180);
        while(count<getColumn())
        {
            bot.motorRB.setPower(-0.2);
            bot.motorRF.setPower(0.2);
            bot.motorLB.setPower(-0.2);
            bot.motorLF.setPower(0.2);
            if(bot.colorsensor2.blue()>10||bot.colorsensor2.red()>10)
            {
                runtime.reset();
                count++;
                while(runtime.seconds()<500)
                {
                    bot.motorRB.setPower(-0.1);
                    bot.motorRF.setPower(0.1);
                    bot.motorLB.setPower(-0.1);
                    bot.motorLF.setPower(0.1);
                }
            }
        }
        stopBotMotors();
        sleep(1000);

        //releases the glyph and pushes the glyph into the cryptobox
        score();
    }
}
