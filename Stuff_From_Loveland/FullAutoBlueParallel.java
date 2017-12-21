package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by WHHS Robotics on 9/23/17.
 */
@Autonomous(name = "FullAutoBlueParallel" , group ="Concept")

public class FullAutoBlueParallel extends Processor{

    @Override
    public void runOpMode() throws InterruptedException {
        //initalizes hardware on robot
        bot.init(hardwareMap);

        //anslyzes the pictogram image
        //checkCol();
        waitForStart();

        //analyzes the pictogram image again
        checkCol();

        //stores the Pictogram image in a instance variable
        //checkVu();

        //sets servo to grab the glyph touching the robot at the start of autonomous
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(1000);

        runtime.reset();
        while(runtime.milliseconds()<300) {
            //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        //knocks the correct jewel off according to our alliance color
        knockJewel(false);

        //moves the robot a distance of 18 inches at an angle of 180 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal
        goAngle(30, 180);

        sleep(1000);

        //turns the robot 180 degrees counter clock wise
        turn(180);

        //moves the robot a very small increment to line up with the cryptobox
        goAngle(13.1, 180);

        sleep(1000);

        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        //gotoColumnRightEnc();

        //stops all motion

        stopBotMotors();
        sleep(1000);

        //releases the glyph and pushes the glyph into the cryptobox
        score();
    }
}



