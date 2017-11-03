package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sushr on 10/28/2017.
 */
@Autonomous(name = "AutoPathScrimmage1234. ", group = "range sensor")
public class AutoPathScrimmage extends TestProcessor{
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


        bot.runtime.reset();
        while(bot.runtime.seconds()>6)
        {

        }
        bot.runtime.reset();

        encoderDrive(DRIVE_SPEED,  13,-13,13,-13, 10.0);  // Moves forward
        turn(-90);
        //encoderDrive(TURN_SPEED,   -22*Math.PI/5.5, 22*Math.PI/5.5, -22*Math.PI/5.5, 22*Math.PI/5.5, 10.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
