package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.hitechnic.*;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.*;

/**
 * Created by Sushruth on 10/19/16.
 */

public class HardwareTileRobot{


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor slideMotorLeft = null;
    public DcMotor slideMotorRight = null;
    public DcMotor intakeMotor = null;
    public DcMotor shooterMotor = null;
    public Servo unicorn = null;
    //public HiTechnicNxtUltrasonicSensor face = null;
    public Servo unicorn2 = null;
    public Servo releaseServo = null;
    public Servo linearServo1 = null;
    public Servo linearServo2 = null;


    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cColorSensor colorSensor = null;
    ModernRoboticsAnalogOpticalDistanceSensor lightSensorRight=null;
    ModernRoboticsAnalogOpticalDistanceSensor lightSensorLeft=null;
    ModernRoboticsTouchSensor touch = null;



    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight = hwMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        slideMotorLeft = hwMap.dcMotor.get("slideMotorLeft");
        slideMotorRight = hwMap.dcMotor.get("slideMotorRight");


        slideMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        slideMotorRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        shooterMotor = hwMap.dcMotor.get("shooterMotor");
        lightSensorRight = (ModernRoboticsAnalogOpticalDistanceSensor)hwMap.opticalDistanceSensor.get("lightSensorRight");
        lightSensorLeft = (ModernRoboticsAnalogOpticalDistanceSensor)hwMap.opticalDistanceSensor.get("lightSensorLeft");


        colorSensor = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("colorSensor");
        touch = (ModernRoboticsTouchSensor) hwMap.touchSensor.get("touch");

        colorSensor.enableLed(false);
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        releaseServo =hwMap.servo.get("releaseServo");
        unicorn = hwMap.servo.get("unicorn");
        unicorn2 = hwMap.servo.get("unicorn2");
        linearServo1 = hwMap.servo.get("linearServo1");
        //linearServo1 is the left linear actuator
        linearServo2 = hwMap.servo.get("linearServo2");

        //face = (HiTechnicNxtUltrasonicSensor) hwMap.ultrasonicSensor.get("face");
        unicorn2.setDirection(Servo.Direction.REVERSE);

        //linearServo2 is the right linear actuator


        //servoLeft.setPosition(0);
        // servoRight.setPosition(0);
        unicorn.setPosition(1.0);
        unicorn.setPosition(1.0);

        linearServo1.setPosition(.15);
        linearServo2.setPosition(.15);
        //slideServo.setPosition(0);
        releaseServo.setPosition(.5);
        //slideServoRight.setPosition(0);
        //servoPusher1.setPosition(0);
        colorSensor.enableLed(false);
        //bottomColorSensor.enableLed(true);



    }



}

