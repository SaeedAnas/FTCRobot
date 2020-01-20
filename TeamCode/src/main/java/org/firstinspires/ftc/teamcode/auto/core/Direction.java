package org.firstinspires.ftc.teamcode.auto.core;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.*;


public enum Direction {

    FORWARD {
        @Override
        public void setPower(double power) {
            topRight.setPower(power);
            topLeft.setPower(power);
            bottomRight.setPower(power);
            bottomLeft.setPower(power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() > targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }

        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
        }
    },
    BACKWARD {
        @Override
        public void setPower(double power) {
            topRight.setPower(-power);
            topLeft.setPower(-power);
            bottomRight.setPower(-power);
            bottomLeft.setPower(-power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() < targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }

        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
        }
    },
    // TODO Change the getTarget and hasNotReached
    LEFT {
        @Override
        public void setPower(double power) {
            topRight.setPower(power);
            topLeft.setPower(-power);
            bottomRight.setPower(-power);
            bottomLeft.setPower(power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                if (i < 2)
                    targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
                else
                    targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if ((i < 2 && motors[i].getCurrentPosition() < targets[i]) || (i >= 2 && motors[i].getCurrentPosition() > targets[i])) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }

        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{bottomRight, topLeft, bottomLeft, topRight};
        }
    },
    // TODO Change the getTarget and hasNotReached
    RIGHT {
        @Override
        public void setPower(double power) {
            topRight.setPower(-power);
            topLeft.setPower(power);
            bottomRight.setPower(power);
            bottomLeft.setPower(-power);
        }
        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                if (i < 2)
                    targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
                else
                    targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if ((i < 2 && motors[i].getCurrentPosition() < targets[i]) || (i >= 2 && motors[i].getCurrentPosition() > targets[i])) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }
        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, bottomLeft, bottomLeft, topRight};
        }
    },
    // TODO Change the getTarget and hasNotReached
    FORWARD_LEFT {
        @Override
        public void setPower(double power) {
            topRight.setPower(power);
            bottomLeft.setPower(power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() > targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }
        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, bottomLeft};
        }
    },
    // TODO Change the getTarget and hasNotReached
    FORWARD_RIGHT {
        @Override
        public void setPower(double power) {
            topLeft.setPower(power);
            bottomRight.setPower(power);
        }
        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() > targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }
        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topLeft, bottomRight};
        }
    },
    // TODO Change the getTarget and hasNotReached
    BACKWARD_RIGHT {
        @Override
        public void setPower(double power) {
            topRight.setPower(-power);
            bottomLeft.setPower(-power);
        }
        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() < targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }
        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, bottomLeft};
        }
    },
    // TODO Change the getTarget and hasNotReached
    BACKWARD_LEFT {
        @Override
        public void setPower(double power) {
            topLeft.setPower(-power);
            bottomRight.setPower(-power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[motors.length];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
            }
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = true;
            for (int i = 0; i < targets.length; i++) {
                if (motors[i].getCurrentPosition() < targets[i]) {
                    hasNotReached = false;
                    break;
                }
            }
            return hasNotReached;
        }

        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topLeft, bottomRight};
        }
    };


    public abstract void setPower(double power);

    public abstract double[] getTarget(double distance, DcMotor[] motors);

    public abstract boolean hasNotReached(double[] targets, DcMotor[] motors);

    public abstract DcMotor[] getMotors();

    public static void stopRobot(DcMotor[] motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

}
//public enum Direction {
//
//    FORWARD {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//        @Override
//        public void setPower(double power) {
//            topRight.setPower(power);
//            topLeft.setPower(power);
//            bottomRight.setPower(power);
//            bottomLeft.setPower(power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double target, DcMotor[] motors) {
//            double motorAvg = getAvg(motors);
//            return motorAvg < target;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
//        }
//    },
//    BACKWARD {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//        @Override
//        public void setPower(double power) {
//            topRight.setPower(-power);
//            topLeft.setPower(-power);
//            bottomRight.setPower(-power);
//            bottomLeft.setPower(-power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double target, DcMotor[] motors) {
//            return getAvg(motors) > target;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
//        }
//    },
//    // TODO Change the getTarget and hasNotReached
//    FORWARD_LEFT {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//        @Override
//        public void setPower(double power) {
//            topRight.setPower(power);
//            bottomLeft.setPower(power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double target, DcMotor[] motors) {
//            double motorAvg = getAvg(motors);
//            return motorAvg < target;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topRight, bottomLeft};
//        }
//    },
//    // TODO Change the getTarget and hasNotReached
//    FORWARD_RIGHT {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//        @Override
//        public void setPower(double power) {
//            topLeft.setPower(power);
//            bottomRight.setPower(power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double target, DcMotor[] motors) {
//            double motorAvg = getAvg(motors);
//            return motorAvg < target;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topLeft, bottomRight};
//        }
//    },
//    // TODO Change the getTarget and hasNotReached
//    BACKWARD_LEFT {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//        @Override
//        public void setPower(double power) {
//            topRight.setPower(-power);
//            bottomLeft.setPower(-power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double targets, DcMotor[] motors) {
//            double motorAvg = getAvg(motors);
//            return motorAvg > targets;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topRight, bottomLeft};
//        }
//    },
//    // TODO Change the getTarget and hasNotReached
//    BACKWARD_RIGHT {
//        final double CORRECTION = 0 * COUNTS_PER_INCH;
//
//        @Override
//        public void setPower(double power) {
//            topLeft.setPower(-power);
//            bottomRight.setPower(-power);
//        }
//
//        @Override
//        public double getTarget(double distance, DcMotor[] motors) {
//            return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
//        }
//
//        @Override
//        public boolean hasNotReached(double target, DcMotor[] motors) {
//            double motorAvg = getAvg(motors);
//            return motorAvg > target;
//        }
//
//        @Override
//        public DcMotor[] getMotors() {
//            return new DcMotor[]{topLeft, bottomRight};
//        }
//    };
//
//
//
//    /**
//     * moves the motors to go to a specific direction
//     * @param power
//     */
//    public abstract void setPower(double power);
//
//    /**
//     * finds the target tick count the motors should get to
//     * @param distance
//     * @param motors
//     * @return
//     */
//    public abstract double getTarget(double distance, DcMotor[] motors);
//
//    /**
//     * determines whether or not the robot has reached the target
//     * @param targets
//     * @param motors
//     * @return
//     */
//    public abstract boolean hasNotReached(double targets, DcMotor[] motors);
//
//    /**
//     * gets the motors that are in use for the direction
//     * @return
//     */
//    public abstract DcMotor[] getMotors();
//
//    /**
//     * stops the motors
//     * @param motors
//     */
//    public static void stopRobot(DcMotor[] motors) {
//        for (DcMotor motor : motors) {
//            motor.setPower(0);
//        }
//    }
//
//}
