package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by wolfie on 9/1/17.
 */

public class Map {
    public BNO055IMU imu;
    HardwareMap hwMap = null;
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor slideMotor;

    Servo glyphServo1;
    Servo glyphServo2;
    Servo glyphServo3;
    Servo glyphServo4;
    Servo jewelServo;

    ColorSensor colorsensor2 = null;

    ColorSensor colorSensor = null;

    DigitalChannel touchSensor;

    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark columnToScore;

    double tX;
    double tY;
    double tZ;

    double rX;
    double rY;
    double rZ;

    float x;
    float y;
    RelicRecoveryVuMark vuMark;

    VuforiaLocalizer vuforia;

    ElapsedTime runtime = new ElapsedTime();


    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;
        imu = hwMap.get(BNO055IMU.class, "imu");
        motorLB = hwMap.dcMotor.get("motorLB");
        motorLF = hwMap.dcMotor.get("motorLF");
        motorRF = hwMap.dcMotor.get("motorRF");
        motorRB = hwMap.dcMotor.get("motorRB");
        slideMotor = hwMap.dcMotor.get("slideMotor");
        //motorLB.setDirection(DcMotor.Direction.REVERSE);
        //^^^^^^^^^^^^^^^^^^^^^^^^^R^RmotorRF.setDirection(DcMotor.Direction.REVERSE);
        glyphServo1 = hwMap.servo.get("glyphServo1");
        glyphServo2 = hwMap.servo.get("glyphServo2");
        glyphServo3 = hwMap.servo.get("glyphServo3");
        glyphServo4 = hwMap.servo.get("glyphServo4");
        jewelServo = hwMap.servo.get("jewelServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        colorsensor2 =  hwMap.get(ColorSensor.class, "colorSensor2");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        touchSensor = hwMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);


        param.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";

        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(param);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        colorSensor.enableLed(true);

        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jewelServo.setPosition(.2);


        glyphServo1.setPosition(0.4);
        glyphServo4.setPosition(.4);
        glyphServo2.setPosition(0.6);
        glyphServo3.setPosition(.6);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}