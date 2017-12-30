package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sushr on 12/15/2017.
 */
@Autonomous(name = "BlueParallel", group = "fjfrjkdk")
public class BlueParallel extends Processor{

    int count = 0;
    boolean touch = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
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
        knockJewel(false);
/*
        while(bot.rangeSensor.getDistance(DistanceUnit.INCH)<getDistanceColumn(getColumn())) {
            telemetry.addData("dist",bot.rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Count: ", count);
            telemetry.update();
            bot.motorRF.setPower(0.2);
            bot.motorRB.setPower(-0.2);
            bot.motorLB.setPower(-0.2);
            bot.motorLF.setPower(0.2);
        }
        */
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

        goAngle(20,180);
        sleep(500);
        align(0);
        sleep(500);
        turn(-180);
        sleep(1000);
        align(180);
        sleep(750);
        raiseColorServo();
        drivingRangeForwardBlue();
        drivingRangeBackBlue();
        drivingRangeForwardBlue();


        gotoColumnLeft();


        sleep(500);
        bot.colorServo.setPosition(0);
        sleep(500);
        align(180);
        //driveToDistance();
        score();
        stopBotMotors();    }
}
