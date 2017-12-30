package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by wolfie on 9/2/17.
 */

public abstract class TestProcessor extends LinearOpMode {
    TestMap bot = new TestMap();
    public final static double DEFAULT_POWER = .7;
    public final static int TICKSPERROTATION = 1120;
    public final static int DIAMETEROFWHEEL = 4;
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
    static final double I_TURN_COEFF = .0;// The coefficients should not be above 1
    static final double P_DRIVE_COEFF = 0.15;
    static final double I_DRIVE_COEFF = 0.1;
    static final double P_COMP_DRIVE_COEFF = 0.1;
    static final double LIGHTING_OFFSET = .1;
    static final double DIS_CENT = -1.75; //inches


   public double getDistance()
   {
       return  (bot.ultrasonicLeft.getVoltage()/3.3*1024);
   }
   public void getTelemetry()
   {
       telemetry.addData("UltrasonicDistance(in)", getDistance());
       telemetry.addData("Voltage ", getVoltagez());
       telemetry.update();
   }
   public double getVoltagez()
   {
       return bot.ultrasonicLeft.getVoltage();
   }
}