package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by wolfie on 9/15/17.
 */
@TeleOp(name = "XbadTele", group = "X")
public class Tele extends OpMode{
    Map bot = new Map();
    double xpow;
    double ypow;
    double zpow;
    double rightx;

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.glyphServo1.setPosition(0.15);
        bot.glyphServo2.setPosition(0.75);
    }

    @Override
    public void loop() {

        //switched the x and z pow
        xpow = gamepad1.left_stick_x;//direction not actually
        ypow = gamepad1.left_stick_y;// variable names are incoorect
        zpow = gamepad1.right_stick_x;

        double mag = Math.sqrt(ypow * ypow + xpow * xpow);
        double theta = Math.atan2(ypow, xpow);
        double aPair = mag * Math.cos(theta - Math.PI/4);
        double bPair = mag * Math.sin(theta - Math.PI/4);

        bot.motorLF.setPower(0.6*(bPair-zpow));
        bot.motorRF.setPower(0.6*(-aPair-zpow));
        bot.motorRB.setPower((0.6*(-bPair-zpow)));
        bot.motorLB.setPower((0.6*(aPair-zpow)));

        double slidePower = gamepad2.left_stick_y;
        double relicPower = gamepad2.right_stick_y;
        //slidePower = slidePower/4;

        bot.slideMotor.setPower(slidePower);


        if(gamepad2.a)
        {
            bot.glyphServo1.setPosition(0.35);
            bot.glyphServo2.setPosition(0.5);
        }
        if (gamepad2.dpad_down)
        {
            bot.glyphServo1.setPosition(1);
            bot.glyphServo2.setPosition(1);
        }
        if (gamepad2.dpad_up)
        {
            bot.glyphServo1.setPosition(0);
            bot.glyphServo2.setPosition(0);
        }
        if(gamepad2.b)
        {
            bot.glyphServo1.setPosition(0.5);
        }
        if(gamepad2.x)
        {
            bot.glyphServo2.setPosition(0.5);
        }
        if(gamepad2.y)
        {
            bot.glyphServo1.setPosition(0.4);
            bot.glyphServo2.setPosition(0.33);
        }
        if(gamepad1.x){
            bot.jewelServo.setPosition(1.0);
        }
        telemetry.addData("motor1Power",bot.slideMotor.getPower());

    }
    public void checkVu() {

        /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
        * it is perhaps unlikely that you will actually need to act on this pose information, but
        * we illustrate it nevertheless, for completeness. */
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) bot.relicTemplate.getListener()).getPose();
        telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            bot.tX = trans.get(0);
            bot.tY = trans.get(1);
            bot.tZ = trans.get(2);

            // X = vertical axis
            // Y = horizonatal Axis
            // Z = Depth Axis
            // Extract the rotational components of the target relative to the robot
            bot.rX = rot.firstAngle;
            bot.rY = rot.secondAngle;
            bot.rZ = rot.thirdAngle;
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        bot.vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
        telemetry.addData("Z", bot.tZ);
        telemetry.addData("X", bot.tX);
        telemetry.update();
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}