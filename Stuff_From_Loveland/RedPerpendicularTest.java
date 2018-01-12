package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/16/2017.
 */
@Autonomous(name = "RedPerpendicularTest", group = "fjfj")
public class RedPerpendicularTest extends Processor{

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
        bot.glyphServo3.setPosition(.08);
        bot.glyphServo4.setPosition(1);
        sleep(500);

        runtime.reset();

        //raises the Rev slides to pick the glyph off the ground to prevent dragging the glyph
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.27);
        sleep(700);

        //knocks the correct jewel off according to our alliance color
        knockJewel(true);
/*
        while(bot.rangeSensor.getDistance(DistanceUnit.INCH)<15) {
            telemetry.addData("dist",bot.rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Count: ", count);
            telemetry.update();
            bot.motorRF.setPower(-0.2);
            bot.motorRB.setPower(0.2);
            bot.motorLB.setPower(0.2);
            bot.motorLF.setPower(-0.2);
        }
        bot.y = angularOffset();
        double difference = bot.x-bot.y;
        turn(difference);*/
        goAngle(20,0);
        sleep(500);
        align(0);
        sleep(500);
        turn(-90);
        sleep(500);
        align(-90);
        sleep(500);
        align(-90);
        raiseColorServo();

        gotoColumnLeft();

        stopBotMotors();

        sleep(500);
        goAngle(3,0);
        bot.colorServo.setPosition(0);
        sleep(500);
        align(-90);
        //releases the glyph and pushes the glyph into the cryptobox

        stopBotMotors();
    }
}