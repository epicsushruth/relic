package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "teletest", group = "X")
public class TeleTest extends OpMode{
    TeleMap bot = new TeleMap();
    double xpow;
    double ypow;
    double zpow;
    double rightx;
    boolean toggle = false;

    @Override
    public void init() {
        //initalizes hardware map
        bot.init(hardwareMap);
    }

    public void readGamePad() {
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = gamepad1.left_stick_y;
        xpow = gamepad1.left_stick_x;

        //creates a deadzone for left stick y
        if(Math.abs(ypow)<.05){
            ypow = 0;

        }
        //creates a deadzone for left stick x
        if(Math.abs(xpow)<.05){
            xpow = 0;

        }
    }

    @Override
    public void loop() {

        //takes the joystick values and converts to motor speeds through holonomic calculations
        readGamePad();
        double mag = Math.sqrt(ypow * ypow + xpow * xpow);
        double theta = Math.atan2(ypow, xpow);
        double aPair = mag * Math.cos(theta - Math.PI/4);
        double bPair = mag * Math.sin(theta - Math.PI/4);


        //sets movement speeds for motors to move correctly based on joystick input
        //runs at .8 speed to provide driver assisting controls
        bot.motorLF.setPower(.8*(bPair-toggle(toggle,zpow)));
        bot.motorRF.setPower(.8*(-aPair-toggle(toggle,zpow)));
        bot.motorRB.setPower(.8*(-bPair-toggle(toggle,zpow)));
        bot.motorLB.setPower(.8*(aPair-toggle(toggle,zpow)));

        //assings the joystick value to another variable
        double slidePower = -gamepad2.left_stick_y;

        if(slidePower>0)
        {
            //scales the slidepower to move at a quarter speed
            slidePower /= 4;
        }
        bot.slideMotor.setPower(slidePower);

        if(gamepad1.a){
            if(!toggle){
                toggle = true;
            }
            else {
                toggle = false;
            }

        }


        //assigns the value of the joystick to a variable
        double relicPower = gamepad2.right_stick_y;

        //sets the variable value to move the motor at the specified speed
        bot.relicMotor.setPower(relicPower);

        if(gamepad2.a)  //closes the servos to hold the glyph
        {
            gripGlyph();
        }
        if(gamepad2.x)  //opens the right servo
        {
            closeJewel();
        }
        if(gamepad2.b)  // openRight
        {
            bot.glyphServo2.setPosition(.8);

            bot.glyphServo1.setPosition(.2);
        }
        if(gamepad2.y) //releases the glyph from the servos
        {
            realeaseGlyph();
        }
        if(gamepad2.dpad_left){
            fingersClose();  // fingers closed for relic
        }
        if(gamepad2.dpad_right){
            fingersOpen(); // opens finger servo for relic
        }
        if(gamepad2.dpad_up){
            wristUp();   // brings wrist up for relic
        }
        if(gamepad2.dpad_down){
            wristDown(); // bring wrist down for relic
        }
        while(gamepad1.x)
        {
            bot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(true) {
                bot.motorLB.setPower(0.5);
                telemetry.addData("encoders", bot.motorLB.getCurrentPosition());
                telemetry.update();
            }
        }
        while(gamepad1.y)
        {
            bot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(true) {
                bot.motorRB.setPower(0.5);
                telemetry.addData("encoders", bot.motorRB.getCurrentPosition());
                telemetry.update();
            }
        }
        while(gamepad1.b)
        {
            bot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(true) {
                bot.motorLF.setPower(0.5);
                telemetry.addData("encoders", bot.motorLF.getCurrentPosition());
                telemetry.update();
            }
        }

        {
            bot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(true) {
                bot.motorRF.setPower(0.5);
                telemetry.addData("encoders", bot.motorRF.getCurrentPosition());
                telemetry.update();
            }
        }

    }

    public double toggle(boolean toggle, double power){
        if(toggle){
            return power * .4;
        }
        else{
            return power;
        }

    }

    public void fingersOpen(){
        bot.relicFingers.setPosition(.6);
    }

    public void fingersClose(){
        bot.relicFingers.setPosition(.95);
    }

    public void wristUp() {
        bot.relicWrist.setPosition(.7);
    }

    public void wristDown() {
        bot.relicWrist.setPosition(0);
    }
    public void gripGlyph() {
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
    }

    public void openRight() {
        bot.glyphServo1.setPosition(0.53);
    }

    public void openLeft() {
        bot.glyphServo2.setPosition(0.5);

    }
    public void closeJewel()
    {
        bot.jewelServo.setPosition(0.2);
    }

    public void realeaseGlyph() {
        openLeft();
        openRight();
    }


}