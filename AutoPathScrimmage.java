package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //motorRF,motorLF,motorRB,motorLB

        checkCol();
/*
        time.reset();
        while(time.seconds()<1.5) {
            bot.glyphServo1.setPosition(0.4);
            bot.glyphServo2.setPosition(0.429);
        }
        time.reset();
        while(time.seconds()<1)
        {
            bot.slideMotor.setPower(-1);
            bot.slideMotor2.setPower(-1);
        }
*/
        //encoderDrive(0.3,   -22*Math.PI/20, -22*Math.PI/20, -22*Math.PI/20, -22*Math.PI/20, 10.0);
        //
        // encoderDrive(0.3,   22*Math.PI/20, 22*Math.PI/20,  22*Math.PI/20, 22*Math.PI/20, 10.0);
        time.reset();
        while(time.seconds()<1)
        {

        }


        encoderDrive(DRIVE_SPEED,  15,-15,15,-15, 10.0);  // Moves forward
        encoderDrive(TURN_SPEED,   -22*Math.PI/5.5, -22*Math.PI/5.5, -22*Math.PI/5.5, -22*Math.PI/5.5, 10.0);
        gotoColumnRight();
        encoderDrive(DRIVE_SPEED,15,-15,15,-15,5.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
