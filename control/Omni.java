package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Created by Sushr on 11/11/2017.
 */

public class Omni {

    static final double P_TURN_COEFF = .2;
    public final static int DIAMETEROFWHEEL = 4;
    static final double TURN_SPEED = 0.3;
    static final double DRIVE_SPEED = 0.6;
    static final double HEADING_THRESHOLD = 2;

    public static class Wheels {
        public final double leftBackPower;
        public final double rightBackPower;
        public final double leftFrontPower;
        public final double rightFrontPower;


        public Wheels(double frontLeft, double frontRight, double backLeft, double backRight)
        {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                    backLeft, backRight);
            clampPowers(powers);

            this.leftFrontPower = powers.get(0);
            this.rightFrontPower = powers.get(1);
            this.leftBackPower = powers.get(2);
            this.rightBackPower = powers.get(3);
        }
        public Wheels scaleWheelPower(double scalar) {
            return new Wheels(this.leftFrontPower * scalar, this.rightFrontPower * scalar,
                    this.leftBackPower * scalar, this.rightBackPower * scalar);
        }
    }
        public static Wheels motionToWheels(Motion motion) {
            double vD = motion.vD;
            double thetaD = motion.thetaD;
            double vTheta = motion.vTheta;

            thetaD = thetaD*180/Math.PI;
            double x = Math.cos(thetaD-Math.PI/4);
            double y = Math.sin(thetaD-Math.PI/4);
            double z = vTheta;
            Wheels motion2 = new Wheels ((vD*(y-x-z)),(-vD*(y-x-z)),(vD*(-y+x-z)),(vD*(y+x-z)));
            return motion2;
        }

        private static void clampPowers(List<Double> powers) {
            double minPower = Collections.min(powers);
            double maxPower = Collections.max(powers);
            double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

            if (maxMag > 1.0) {
                for (int i = 0; i < powers.size(); i++) {
                    powers.set(i, powers.get(i) / maxMag);
                }
            }
        }

        public static class TurnSpeed{


            double error;
            public TurnSpeed(double first,double wanted)
            {
                error = wanted - first;
            }

            public double turning(double firstAngle, double angleWanted) {
                double correction;
                double speed;
                while (error > 180)
                    error -= 360;
                while (error < -180)
                    error += 360;

                correction = Range.clip( error * P_TURN_COEFF,-1,1);



                if(Math.abs(error) <= HEADING_THRESHOLD){
                    return 0;
                }
                else{
                    speed = TURN_SPEED * correction;
                }
                return speed;
            }
        }

        public static class Motion {
            // Robot speed [-1, 1].
            public final double vD;
            // Robot angle while moving [0, 2pi].
            public final double thetaD;
            // Speed for changing direction [-1, 1].
            public final double vTheta;

            /**
             * Sets the motion to the given values.
             */
            public Motion(double vD, double thetaD, double vTheta) {
                this.vD = vD;
                this.thetaD = thetaD;
                this.vTheta = vTheta;
            }
        }
            //fix this teleop joystick motion
            public static Motion joystickToMotion(double leftStickX,
                                                  double leftStickY,
                                                  double rightStickX,
                                                  double rightStickY) {
                double xpow = leftStickX;
                double ypow = leftStickY;
                double zpow = -rightStickX;

                double vD = Math.sqrt(ypow * ypow + xpow * xpow);
                double theta = Math.atan2(ypow, xpow);


                return new Motion(vD, theta, zpow);
            }

    }


