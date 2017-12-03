package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by WHHS robotics on 9/23/17.
 */
@Autonomous(name = "FullAutoRedParallel" , group ="Concept")

public class FullAutoRedParallel extends Processor{

    @Override
    public void runOpMode() throws InterruptedException {
        //initializes all hardware on robot
        bot.init(hardwareMap);

        //analyzes the Pictogram image
       // checkCol();
        waitForStart();

        //analyzes the Pictogram image
        checkCol();

        //stores the Pictogram image value in a instance variable
        checkVu();

        //sets servo to grab the glyph touching the robot at the start of autonomous
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(2000);

        runtime.reset();
        while(runtime.milliseconds()<300) {
            //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);

        //moves the robot a distance of 30 inches at an angle of 0 off the horizontal with the side with the glyph servo being orientated at the angle of 90 off the horizontal
        goAngle(30, 0);

        sleep(500);


        //turns the robot 177 degrees counter clock wise
        turn(177);

        //moves the robot a very small increment to line up with the cryptobox
        goAngle(12.5, 0);

        sleep(500);

        //travels in increments along the cryptobox to stop at the correct column indicated by the Pictogram image
        gotoColumnLeftEnc();

        //stops all motion
        stopBotMotors();
        sleep(500);

        //releases the glyph and pushes the glyph into the cryptobox
        score();


    }
}