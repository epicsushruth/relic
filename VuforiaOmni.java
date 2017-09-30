package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Sushr on 9/8/2017.
 */
@Autonomous(name = "vuforiaomnitest", group = "tiles")
public class VuforiaOmni extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftDrive = hardwareMap.dcMotor.get("left drive");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotor rightDrive = hardwareMap.dcMotor.get("right drive");
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotor frontDrive = hardwareMap.dcMotor.get("front drive");
        frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotor backDrive = hardwareMap.dcMotor.get("back drive");
        backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        backDrive.setDirection(DcMotor.Direction.FORWARD);
        frontDrive.setDirection(DcMotor.Direction.REVERSE);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4\n";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2017-2018");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setPower(0.2);
        rightDrive.setPower(0.2);

        while (opModeIsActive() && wheels.getRawPose() == null) {
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);


    VectorF angles = anglesFromTarget(wheels);

    VectorF trans = navOffWall(wheels.getPose().getTranslation(),Math.toDegrees(angles.get(0))-90, new VectorF(500,0,0));
        //50 cm off the wall no side to side change or y
    if(trans.get(0)>0){

        leftDrive.setPower(0.02);
        rightDrive.setPower(-0.02);
    } else {
        leftDrive.setPower(-0.02);
        rightDrive.setPower(0.02);
    }

    do{
        if(wheels.getPose() != null)
        {
            trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500,0,0));
        }
        idle();
    } while(opModeIsActive() && Math.abs(trans.get(0))> 30);

     leftDrive.setPower(0);
     rightDrive.setPower(0);

     leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

     leftDrive.setTargetPosition((int)(leftDrive.getCurrentPosition() + ((Math.hypot(trans.get(0),trans.get(2))+80)/319.185813605*1120)));
     rightDrive.setTargetPosition((int)(rightDrive.getCurrentPosition() + ((Math.hypot(trans.get(0),trans.get(2))+80)/319.185813605*1120)));

     leftDrive.setPower(0.3);
     rightDrive.setPower(0.3);

        while(opModeIsActive() &&leftDrive.isBusy()&&rightDrive.isBusy())
        {
            idle();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    while(opModeIsActive()&&(wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0))>10)) {
        if (wheels.getPose() != null) {
            if (wheels != null) {
                leftDrive.setPower(-0.3);
                rightDrive.setPower(0.3);
            } else {
                leftDrive.setPower(0.3);
                rightDrive.setPower(-0.3);
            }
        } else {
                leftDrive.setPower(-0.3);
                rightDrive.setPower(0.3);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }



    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }
    //analyze beacon here
}





