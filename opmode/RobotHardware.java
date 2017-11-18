package org.firstinspires.ftc.teamcode.opmode;
import java.util.ArrayList;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.control.Omni;


/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 * Per-robot customization configured in SharedCode/src/main/res/values/.
 */
public abstract class RobotHardware extends OpMode {
    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
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
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }


    protected void setDriveForOmni(Omni.Motion motion) {
        Omni.Wheels wheels = Omni.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.leftFrontPower);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.rightFrontPower);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.leftBackPower);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.rightBackPower);
    }


    protected void setDriveForOmniForSpeed(Omni.Motion motion) {
        Omni.Wheels wheels = Omni.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.leftFrontPower);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.rightFrontPower);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.leftBackPower);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.rightBackPower);
    }


    protected enum ServoName {
        JEWEL_DROP,
        JEWEL_HIT,
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
        setAngle(ServoName.JEWEL_DROP, raisedJewelAngle);
    }

    // Lowers the jewel arm.
    protected void lowerJewelArm() {
        setAngle(ServoName.JEWEL_DROP, loweredJewelAngle);
    }

    // Centers the jewel arm.
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

    // The color sensors on the robot.
    protected enum ColorSensorName {
        JEWEL,
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
        FIELD_CENTER,
        FIELD_CORNER,
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
        raisedJewelAngle = 1.0;
        loweredJewelAngle = 0.0;
        centerJewelAngle = 1.0;
        forwardJewelAngle = 0.3;
        backwardJewelAngle = 0.7;
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

        raiseJewelArm();
        centerJewelArm();
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

    // Per robot tuning parameters.
    private String vuforiaLicenseKey;
    private double raisedJewelAngle;
    private double loweredJewelAngle;
    private double centerJewelAngle;
    private double forwardJewelAngle;
    private double backwardJewelAngle;
}
