package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by wolfie on 9/2/17.
 */

public abstract class Processor extends LinearOpMode {
    Map bot = new Map();
    public final static double DEFAULT_POWER = .7;
    public final static int TICKSPERROTATION = 1120;
    public final static int DIAMETEROFWHEEL = 4;

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

            // X = vertical axis,
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
        telemetry.addData("X", bot.tX);
        telemetry.addData("Y", bot.tZ);
        telemetry.update();
    }
    public void checkCol(){
        checkVu();
        if (bot.vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", bot.vuMark);

            bot.columnToScore = bot.vuMark;
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    // SEPERATE STATE LATER IS SCORE STATE
    // PASS IN PARAMETERS THAT WILL TELL HOW TO SCORE
    public void scoreColumn(){

        if(bot.columnToScore == RelicRecoveryVuMark.UNKNOWN){
            return;
        }
        if(bot.columnToScore == RelicRecoveryVuMark.CENTER){
            score();
        }
        if(bot.columnToScore == RelicRecoveryVuMark.LEFT){
            score();
        }
        if(bot.columnToScore == RelicRecoveryVuMark.RIGHT){
            score();
        }
    }

    public void score(){
        //MOTOR MOTIONS TO SCORE
    }

    public void getOffStone(){
        //accesses the gyro values
        //drive based on Vuforia in
    }

    public void drive(){

    }
    // THIS SHOULD BE A STATE
    public void faceImage(){
        checkVu();
        while(bot.vuMark == RelicRecoveryVuMark.UNKNOWN){
            bot.vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
            bot.motorRB.setPower(.1);
            bot.motorRF.setPower(.1);
            bot.motorLF.setPower(.1);
            bot.motorLB.setPower(.1);
            checkVu();
        }
        bot.motorRB.setPower(0);
        bot.motorRF.setPower(0);
        bot.motorLF.setPower(0);
        bot.motorLB.setPower(0);
    }

    public void go(double targetX, double targetZ){
        double a;
        double b;
        double c;

        double y;
        double x;
        double z;
        double angleV1;

        a = targetX - bot.tX;
        b = targetZ - bot.tZ;

        double p = .01; //correction factor;
        bot.vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
        while(Math.abs(a)>(50)|| Math.abs(b)>(50)){
            a = targetX - bot.tX;
            b = targetZ - bot.tZ;
            c = Math.sqrt(a*a+b*b);

            angleV1 = Math.atan((bot.tX/bot.tZ));

            y = b/c;
            x= a/c;
            z= p*((bot.rY*Math.PI/180)+(angleV1));

            bot.motorLF.setPower(Range.clip((y-x-z)/2,-1,1));
            bot.motorRF.setPower(Range.clip((-y-x-z)/2,-1,1));
            bot.motorRB.setPower(Range.clip((y+x-z)/2,-1,1));
            bot.motorLB.setPower(Range.clip((-y+x-z)/2,-1,1));

            telemetry.addData("a%f",a);
            telemetry.addData("bot X",bot.tX );
            telemetry.addData("target X", targetX);
            telemetry.addData("target X - bot X", targetX - bot.tX);
            telemetry.addData("b%f",b);
            telemetry.addData("bot Z",bot.tZ);
            telemetry.addData("target Z", targetZ);
            telemetry.addData("target Z - bot Z", targetZ - bot.tZ);
            telemetry.addData("c%f",c);
            telemetry.addData("x%f",x);
            telemetry.addData("y%f",y);
            telemetry.addData("angleV1%f",angleV1);


            checkVu();


        }

        bot.motorLF.setPower(0);
        bot.motorRF.setPower(0);
        bot.motorRB.setPower(0);
        bot.motorLB.setPower(0);

    }
    public void goToTarget(double x, double y)
    {

        double[] angle = new double[18];

        for(int j= 0;j<angle.length;j++)
        {
            angle[j] = .0559*j;
        }

        double robotTranslationY = bot.tZ;
        double robotTranslationX = bot.tX;

        int index =0;

        double yCoordinate = y - robotTranslationY;
        double xCoordinate = x - robotTranslationX;
        double robotBearing;
        double targetRange;
        double targetBearing;
        double relativeBearing;
        //!(Math.abs(yCoordinate)>(70) ^ Math.abs(xCoordinate)>(70))

        robotBearing = bot.rZ;

        // target range is based on distance from robot position to origin.
        targetRange = Math.hypot(bot.tX, bot.tY);

        // target bearing is based on angle formed between the X axis to the target range line
        targetBearing = Math.toDegrees(-Math.asin(bot.tY / targetRange));

        // Target relative bearing is the target Heading relative to the direction the robot is pointing.
        relativeBearing = targetBearing - robotBearing;
/*
        while(Math.abs(relativeBearing)>10)
        {
            bot.motorLB.setPower(0.1);
            bot.motorLF.setPower(0.1);
            bot.motorRB.setPower(0.1);
            bot.motorRF.setPower(0.1);
            checkVu();

            robotBearing = bot.rZ;
            // target range is based on distance from robot position to origin.
            targetRange = Math.hypot(bot.tX, bot.tZ);

            // target bearing is based on angle formed between the X axis to the target range line
            targetBearing = Math.toDegrees(-Math.asin(bot.tZ / targetRange));

            // Target relative bearing is the target Heading relative to the direction the robot is pointing.
            relativeBearing = targetBearing - robotBearing;
        }
*/
        while(Math.abs(xCoordinate)>(50)|| Math.abs(yCoordinate)>(50)) {
            robotTranslationX = bot.tX;
            robotTranslationY = bot.tZ;

            yCoordinate = y - robotTranslationY;
            xCoordinate = x - robotTranslationX;
            double angleV1 = Math.atan((bot.tX/bot.tZ ));
            if(bot.tZ<0)
            {
                angleV1 += 180;
            }

            //double speedA = 0.3;
            double speedZ = 0;
            if (angleV1>17)
            {
                index = 17;
            }
            else
            {
                index = (int) angleV1;
            }
            speedZ = angle[index];
            speedZ /= 60;
            speedZ = 0;
            //double speedB = ((speedA * (yCoordinate - xCoordinate)) / (yCoordinate + xCoordinate));
            double ang = Math.atan2(yCoordinate, xCoordinate) - Math.PI/4;
            double norm = Math.abs(Math.sin(ang)) + Math.abs(Math.cos(ang));
            norm = 4*Math.sqrt(yCoordinate*yCoordinate+xCoordinate*xCoordinate);
            double ynorm = yCoordinate/norm;
            double xnorm = xCoordinate/norm;
            bot.motorRF.setPower((ynorm - xnorm));
            telemetry.addData("MotorRF",bot.motorRF.getPower());
            bot.motorLB.setPower(-(ynorm -  xnorm));
            telemetry.addData("MotorLB",bot.motorLB.getPower());
            bot.motorLF.setPower(-(ynorm + xnorm));
            telemetry.addData("MotorLF",bot.motorLF.getPower());
            bot.motorRB.setPower((ynorm + xnorm));
            telemetry.addData("MotorRB",bot.motorRB.getPower());
            telemetry.addData("motorRFPower", bot.motorRF.getPower());
            telemetry.addData("motorLFPower", bot.motorLF.getPower());
            telemetry.addData("motorRBPower", bot.motorRB.getPower());
            telemetry.addData("motorRFPower", bot.motorRF.getPower());
            telemetry.addData("ynorm", ynorm);
            telemetry.addData("xnorm",xnorm);
            telemetry.addData("motorRBPower", bot.motorRF.getPower());
            telemetry.addData("motorRBPower", bot.motorRF.getPower());
            telemetry.addData("motorRBPower", bot.motorRF.getPower());

            checkVu();

        }
        stopAllMotors();
    }
        public void stopAllMotors(){
            bot.motorLB.setPower(0);
            bot.motorLF.setPower(0);
            bot.motorRB.setPower(0);
            bot.motorRF.setPower(0);
        }

}
