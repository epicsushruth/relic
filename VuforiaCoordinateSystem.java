package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

/**
 * Created by Sushruth on 9/22/2017.
 */
@Autonomous(name = "autonomousVuforiaCoordinates", group = "fgfgfgf")
public class VuforiaCoordinateSystem extends LinearOpMode{
    public static final String TAG = "Vuforia Navigation Sample";


    VuforiaCoordinateSystemHardwareMap robot = new VuforiaCoordinateSystemHardwareMap();
    VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;
    OpenGLMatrix robotLocationTransform = null;
    VectorF trans = null;
    Orientation rot =null;

    double robotX;
    double robotY;
    double robotZ;
    List<VuforiaTrackable> allTrackables;

    RelicRecoveryVuMark vuMark;

    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    double rX;
    double rY;
    double rZ;
    double angleV1;
    OpenGLMatrix pose = null;

    double[] angle = new double[18];
    int scalar = 5;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //allows camera to go to display
        params.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //uses back
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        this.vuforia = ClassFactory.createVuforiaLocalizer(params);

        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");


        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //gets position of the pictogram

        allTrackables = new ArrayList<>();
        allTrackables.addAll(relicTrackables);


        waitForStart();

        relicTrackables.activate();


        while (opModeIsActive()) {

            checkVumark();

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();


                pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                if (pose != null) {

                    // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                    trans = robotLocationTransform.getTranslation();
                    rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    // Robot position is defined by the standard Matrix translation (x and y)
                    robotX = trans.get(0);
                    robotY = trans.get(1);
                    robotZ = trans.get(2);
                    // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                telemetry.addData("Robot (X)", robotX);
                telemetry.addData("Robot (Y)", robotY);
                telemetry.addData("Robot Bearing", rZ);
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();

            goToTarget(lastLocation.getTranslation().get(0) + 20, lastLocation.getTranslation().get(1) + 20);
        }
    }

    public void goToTarget(double x, double y)
    {
        for(int j= 1;j<angle.length+1;j++)
        {
            angle[j] = .0559*j;
        }

        while((lastLocation.getTranslation().get(0)!=x+10)||(lastLocation.getTranslation().get(0)!=y+10)) {

            float robotTranslationX = lastLocation.getTranslation().get(0);
            float robotTranslationY = lastLocation.getTranslation().get(1);

            double yCoordinate = y - robotTranslationY;
            double xCoordinate = x - robotTranslationX;

            angleV1 = Math.atan((robotZ/robotY));
            if(robotZ<0)
            {
                angleV1 += 180;
            }

            double speedA = 0.5;
            double speedZ = 0;

            if(angleV1>3)
            {
                speedZ = angle[(int)(angleV1)];
            }

            double speedB = ((speedA * (yCoordinate - xCoordinate)) / (yCoordinate + xCoordinate));

            robot.motorFront.setPower(speedA+speedZ);
            robot.motorBack.setPower(speedA-speedZ);
            robot.motorLeft.setPower(speedB-speedZ);
            robot.motorRight.setPower(speedB+speedZ);

        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }


    public void checkVumark()
    {
        while(vuMark == RelicRecoveryVuMark.UNKNOWN){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            robot.motorRight.setPower(.5);
            robot.motorFront.setPower(.5);
            robot.motorLeft.setPower(.5);
            robot.motorBack.setPower(.5);

        }
        robot.motorRight.setPower(0);
        robot.motorFront.setPower(0);
        robot.motorLeft.setPower(0);
        robot.motorBack.setPower(0);
    }
}
