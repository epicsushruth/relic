package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
 * Created by Sushr on 9/16/2017.
 */

public abstract class RelicRecoveryMethodHandler extends LinearOpMode{
    RelicRecoveryAutonomousHardwareMap robot = new RelicRecoveryAutonomousHardwareMap();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));
    static final double OMNI_WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    static final double HEADING_THRESHOLD = 1;
    static final double ENCODER_THRESHOLD = 1;

    static final double P_TURN_COEFF = .025;
    static final double I_TURN_COEFF = .0;// The coeffients should not be above 1
    static final double P_DRIVE_COEFF = 0.15;
    static final double I_DRIVE_COEFF = 0.1;
    static final double P_COMP_DRIVE_COEFF = 0.1;
    static final double LIGHTING_OFFSET = .1;
    static final double DIS_CENT = -1.75; //inches


    //distance is in inches
    public void strafeForward(int distance) {
        double temp = COUNTS_PER_MOTOR_REV * (distance / OMNI_WHEEL_CIRCUMFERENCE);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while ((robot.motorBack.getCurrentPosition() < temp + 100) && (robot.motorFront.getCurrentPosition() < temp + 100) && (robot.motorLeft.getCurrentPosition() < temp + 100) && (robot.motorRight.getCurrentPosition() < temp + 100)) {
            robot.motorBack.setPower(-0.5);
            robot.motorFront.setPower(0.5);
            robot.motorLeft.setPower(-0.5);
            robot.motorRight.setPower(0.5);
        }

    }

    public void strafeBack(int distance) {
        double temp = COUNTS_PER_MOTOR_REV * (distance / OMNI_WHEEL_CIRCUMFERENCE);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while ((robot.motorBack.getCurrentPosition() < temp + 100) && (robot.motorFront.getCurrentPosition() < temp + 100) && (robot.motorLeft.getCurrentPosition() < temp + 100) && (robot.motorRight.getCurrentPosition() < temp + 100)) {
            robot.motorBack.setPower(0.5);
            robot.motorFront.setPower(-0.5);
            robot.motorLeft.setPower(0.5);
            robot.motorRight.setPower(-0.5);
        }

    }

    public void strafeLeft(int distance) {
        double temp = COUNTS_PER_MOTOR_REV * (distance / OMNI_WHEEL_CIRCUMFERENCE);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while ((robot.motorBack.getCurrentPosition() < temp + 100) && (robot.motorFront.getCurrentPosition() < temp + 100) && (robot.motorLeft.getCurrentPosition() < temp + 100) && (robot.motorRight.getCurrentPosition() < temp + 100)) {
            robot.motorBack.setPower(-0.5);
            robot.motorFront.setPower(0.5);
            robot.motorLeft.setPower(0.5);
            robot.motorRight.setPower(-0.5);
        }

    }

    public void strafeRight(int distance) {
        double temp = COUNTS_PER_MOTOR_REV * (distance / OMNI_WHEEL_CIRCUMFERENCE);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while ((robot.motorBack.getCurrentPosition() < temp + 100) && (robot.motorFront.getCurrentPosition() < temp + 100) && (robot.motorLeft.getCurrentPosition() < temp + 100) && (robot.motorRight.getCurrentPosition() < temp + 100)) {
            robot.motorBack.setPower(0.5);
            robot.motorFront.setPower(-0.5);
            robot.motorLeft.setPower(-0.5);
            robot.motorRight.setPower(0.5);
        }

    }




}



