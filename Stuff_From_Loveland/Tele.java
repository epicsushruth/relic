package org.firstinspires.ftc.teamcode.Stuff_From_Loveland;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "XTele", group = "X")
public class Tele extends OpMode{
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

        double mag = ypow * ypow + xpow * xpow;
        double theta = Math.round(Math.atan2(ypow, xpow) * 4.0 / Math.PI) * Math.PI / 4.0;
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

        if(gamepad2.right_bumper)  //closes the servos to hold the glyph
        {
            gripGlyphTop();
        }
        if(gamepad2.left_bumper)
        {
            gripGlyphBot();
        }
        if(gamepad2.y) //releases the glyph from the servos
        {
            ram();
        }

        if(gamepad2.b)  //opens the right servo
        {
            realeaseGlyphBot();
            realeaseGlyphTop();
        }
        if(gamepad2.x){
            resetGlpyhpos();
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
        bot.relicFingers.setPosition(1);
    }

    public void fingersClose(){
        bot.relicFingers.setPosition(0);
    }

    public void wristUp() {
        bot.relicWrist.setPosition(0);
    }

    public void wristDown() {
        bot.relicWrist.setPosition(1);
    }
    public void gripGlyphBot() {
        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.27);
    }
    public void gripGlyphTop()
    {
        bot.glyphServo3.setPosition(0.15);
        bot.glyphServo4.setPosition(0.5);
    }

    public void openRightBot() {
        bot.glyphServo1.setPosition(0.53);
    }

    public void openLeftBot() {
        bot.glyphServo2.setPosition(0.42);

    }
    public void openRightTop() {
        bot.glyphServo3.setPosition(.33);
    }

    public void openLeftTop() {
        bot.glyphServo4.setPosition(0.35);

    }
    public void closeJewel()
    {
        bot.jewelServo.setPosition(0.2);
    }

    public void realeaseGlyphBot() {
        openLeftBot();
        openRightBot();
    }
    public void realeaseGlyphTop()
    {
        openLeftTop();
        openRightTop();
    }
    public void ram()
    {
        bot.glyphServo1.setPosition(.28);
        bot.glyphServo2.setPosition(.63);
        bot.glyphServo3.setPosition(.7);
        bot.glyphServo4.setPosition(.18);
    }

    public void resetGlpyhpos(){
        realeaseGlyphBot();
        gripGlyphTop();
    }

}