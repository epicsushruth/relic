package org.firstinspires.ftc.teamcode.control;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Created by Sushr on 11/11/2017.
 */

public class Omni {

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
            double x = Math.cos(thetaD);
            double y = Math.sin(thetaD);
            double z = vTheta;
            Wheels motion2 = new Wheels ((vD*(y+x-z)),(-vD*(y-x+z)),(vD*(y-x-z)),(vD*(y+x+z)));
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
                double vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) +
                                Math.pow(leftStickY, 2)),
                        1);
                double thetaD = Math.atan2(-leftStickX, leftStickY);
                double vTheta = -rightStickX;
                return new Motion(vD, thetaD, vTheta);
            }

    }


