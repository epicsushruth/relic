package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Sushr on 10/28/2017.
 */
@Autonomous(name = "AutoPathScrimmage1234. ", group = "range sensor")
public class AutoPathScrimmage extends Processor{
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        bot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        bot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                bot.motorLB.getCurrentPosition(),
                bot.motorRB.getCurrentPosition(),
                bot.motorLF.getCurrentPosition(),
                bot.motorRF.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();


        //RelicRecoveryVuMark column = detectMark();
        /*bot.glyphServo1.setPosition(0.47);
        bot.glyphServo2.setPosition(0.429);
        sleep(1000);

        knockJewel(false);
        sleep(2000);

        //forward(300);
        // go in front of the cryptograph
        bot.jewelServo.setPosition(1.0);
        sleep(2000);

        encoderDrive(DRIVE_SPEED,  15,15,-15,-15, 10.0);  // Moves forward
        sleep(1000);
        turn(-180);*/
        sleep(3000);
        //driveToColumnBlue(/*columnNumberBlue(column)*/2);
/*
        turn(-90);

        encoderDrive(.4,5,-5,5,-5,10);
        bot.runtime.reset();
        while(bot.runtime.seconds()<2)
        {

        }
        gotoColumnLeft();
        bot.runtime.reset();
        while(bot.runtime.seconds()<1)
        {

        }
        score();
*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
