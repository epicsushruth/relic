package org.firstinspires.ftc.teamcode.opmode;
import android.provider.ContactsContract;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.control.Omni;

import static org.firstinspires.ftc.teamcode.opmode.RobotHardware.ImuName.imu;


/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 * Per-robot customization configured in SharedCode/src/main/res/values/.
 */
public abstract class RobotHardware extends OpMode {
    // The motors on the robot.
    public enum MotorName {
        leftFront,
        rightFront,
        leftBack,
        rightBack,
    }

    public enum ImuName {
        imu
    }


    protected void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setPower(power);
        }
    }


    protected void setDriveForTank(double left, double right) {
        setPower(MotorName.leftFront, left);
        setPower(MotorName.leftBack, left);
        setPower(MotorName.rightFront, right);
        setPower(MotorName.rightBack, right);
    }


    protected void setDriveForOmni(Omni.Motion motion) {
        Omni.Wheels wheels = Omni.motionToWheels(motion);
        setPower(MotorName.leftFront, wheels.leftFrontPower);
        setPower(MotorName.leftBack, wheels.rightFrontPower);
        setPower(MotorName.rightFront, wheels.leftBackPower);
        setPower(MotorName.rightBack, wheels.rightBackPower);
    }
    protected void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
        setPower(MotorName.leftFront, clip_speed);
        setPower(MotorName.leftBack, clip_speed);
        setPower(MotorName.rightFront, clip_speed);
        setPower(MotorName.rightBack, clip_speed);
    }
    protected Orientation getAngularOrientation()
    {
        Orientation ret = allImuSensors.get(0).getAngularOrientation();
        return ret;
    }

    protected void turn(double target, Omni.TurnSpeed speed) {
        Orientation ref = allImuSensors.get(0).getAngularOrientation();
        double heading = ref.firstAngle;
        double angleWanted = target+heading;

        double turnSpeed = speed.turning(ref.firstAngle,angleWanted);
        double correction;
        double error;

        ref = allImuSensors.get(0).getAngularOrientation();
        while(turnSpeed != 0 ){
            ref = allImuSensors.get(0).getAngularOrientation();
            turnSpeed = speed.turning(ref.firstAngle, angleWanted);
            accelerate(turnSpeed);
        }
        accelerate(0);
    }

    protected void setDriveForOmniForSpeed(Omni.Motion motion) {
        Omni.Wheels wheels = Omni.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(MotorName.leftFront, wheels.leftFrontPower);
        setPower(MotorName.leftBack, wheels.rightFrontPower);
        setPower(MotorName.rightFront, wheels.leftBackPower);
        setPower(MotorName.rightBack, wheels.rightBackPower);
    }


    protected enum ServoName {
        jewelServo,
        //JEWEL_HIT,
    }

    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    protected void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
        } else {
            s.setPosition(position);
        }
    }

    // Raises the jewel arm.
    protected void raiseJewelArm() {
        setAngle(ServoName.jewelServo, raisedJewelAngle);
    }

    // Lowers the jewel arm.
    protected void lowerJewelArm() {
        setAngle(ServoName.jewelServo, loweredJewelAngle);
    }

    // Centers the jewel arm.
    /*
    protected void centerJewelArm() {
        setAngle(ServoName.JEWEL_HIT, centerJewelAngle);
    }

    // Moves the jewel arm forward.
    protected void forwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, forwardJewelAngle);
    }

    // Moves the jewel arm backward.
    protected void backwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, backwardJewelAngle);
    }
*/
    // The color sensors on the robot.
    protected enum ColorSensorName {
        colorSensor
    }

    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color channel to read intensity.
     */
    protected int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
            return 0;
        }

        switch (color) {
            case RED: return s.red();
            case GREEN: return s.green();
            case BLUE: return s.blue();
            case ALPHA: return s.alpha();
            default: return 0;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     * @param sensor The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    protected void setColorSensorLedEnabled(ColorSensorName sensor,
                                            boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    // The distance sensors on the robot.
    protected enum DistanceSensorName {
        LEFT,
        FRONT,
        RIGHT,
    }

    /**
     * Gets the distance value on the sensor in centimeters.
     * Returns -1 when the sensor is unavailable.
     * @param sensor The sensor to read.
     */
    protected double getDistanceSensorCm(DistanceSensorName sensor) {
        DistanceSensor s = allDistanceSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Distance Sensor Missing", sensor.name());
            return -1;
        }
        return s.getDistance(DistanceUnit.CM);
    }

    // Possible starting positions.
    protected enum StartPosition {
        FIELD_PARALLEL,
        FIELD_PEREPENDICULAR,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
        return vuforiaLicenseKey;
    }

    /**
     * Initialize the hardware handles.
     */
    public void init() {
        raisedJewelAngle = 0.2;
        loweredJewelAngle = 0.9;

        vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";
        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }

        allDistanceSensors = new ArrayList<DistanceSensor>();
        for (DistanceSensorName s : DistanceSensorName.values()) {
            try {
                allDistanceSensors.add(hardwareMap.get(
                        ModernRoboticsI2cRangeSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Distance Sensor Missing", s.name());
                allDistanceSensors.add(null);
            }
        }

        allImuSensors = new ArrayList<BNO055IMU>();
        for (ImuName s : ImuName.values()) {
            try {
                allImuSensors.add(hardwareMap.get(BNO055IMU.class, s.name()));
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                allImuSensors.get(0).initialize(parameters);

            } catch (Exception e) {
                telemetry.addData("Imu Sensor Missing", s.name());
                allImuSensors.add(null);
            }
        }

        raiseJewelArm();

    }

    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;
    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;
    // All distance sensors on the robot, in order of DistanceSensorName.
    private ArrayList<DistanceSensor> allDistanceSensors;
    // All IMU sensors on the robot, in order of ImuSensorName
    private List<BNO055IMU> allImuSensors;

    // Per robot tuning parameters.
    private String vuforiaLicenseKey;
    private double raisedJewelAngle;
    private double loweredJewelAngle;
    private double centerJewelAngle;
    private double forwardJewelAngle;
    private double backwardJewelAngle;
}
