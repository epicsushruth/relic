package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sushr on 12/21/2017.
 */
@Autonomous(name = "distance", group = "fjfj")
public class testDistance extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("RangeSensor(CM): ",bot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("ProximitySensor(CM)", bot.colorSensor2.getDistance(DistanceUnit.CM));
            telemetry.addData("MaxBotics(In)", getDistanceLeft());
            telemetry.addData("Voltage ", bot.ultrasonicLeft.getVoltage());

            telemetry.update();
        }
    }


    public void lineUP(double distance){
        while(Math.abs(getDistanceLeft()-distance) >= 1 && Math.abs(bot.rangeSensor.getDistance(DistanceUnit.INCH)-distance) >= 1) {
            if(Math.abs(getDistanceLeft()-distance)>Math.abs(bot.rangeSensor.getDistance(DistanceUnit.INCH)-distance)){
                bot.motorRF.setPower(.1);
                bot.motorLF.setPower(.1);
                bot.motorLB.setPower(.1);
                bot.motorRB.setPower(.1);
            }
            if(Math.abs(getDistanceLeft()-distance)<Math.abs(bot.rangeSensor.getDistance(DistanceUnit.INCH)-distance)){
                bot.motorRF.setPower(-.1);
                bot.motorLF.setPower(-.1);
                bot.motorLB.setPower(-.1);
                bot.motorRB.setPower(-.1);
            }
        }


    }

}
