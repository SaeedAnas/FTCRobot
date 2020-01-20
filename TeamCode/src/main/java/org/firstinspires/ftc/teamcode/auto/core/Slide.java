package org.firstinspires.ftc.teamcode.auto.core;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.*;

public enum Slide {
    // TODO Change the getTarget and hasNotReached
    LEFT {
        final double CORRECTION = 0 * COUNTS_PER_INCH;
        @Override
        public void setPower(double power) {
            topRight.setPower(power);
            topLeft.setPower(-power);
            bottomRight.setPower(-power);
            bottomLeft.setPower(power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[2];
            targets[0] = getAvg(new DcMotor[] {motors[1], motors[2]}) + (COUNTS_PER_INCH * distance) + CORRECTION;
            targets[1] = getAvg(new DcMotor[] {motors[0], motors[3]}) - (COUNTS_PER_INCH * distance) - CORRECTION;
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = false;
            double currentPos = getAvg(new DcMotor[] {motors[1], motors[2]});
            double currentNeg = getAvg(new DcMotor[] {motors[0], motors[3]});
            if (targets[0] > currentPos || targets[1] < currentNeg) {
                hasNotReached = true;
            }
            return hasNotReached;
        }


        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topLeft, topRight, bottomLeft , bottomRight};
        }
    },
    // TODO Change the getTarget and hasNotReached
    RIGHT {
        final double CORRECTION = 0 * COUNTS_PER_INCH;
        @Override
        public void setPower(double power) {
            topRight.setPower(-power);
            topLeft.setPower(power);
            bottomRight.setPower(power);
            bottomLeft.setPower(-power);
        }

        @Override
        public double[] getTarget(double distance, DcMotor[] motors) {
            double[] targets = new double[2];
            targets[0] = getAvg(new DcMotor[] {motors[1], motors[2]}) + (COUNTS_PER_INCH * distance) + CORRECTION;
            targets[1] = getAvg(new DcMotor[] {motors[0], motors[3]}) - (COUNTS_PER_INCH * distance) - CORRECTION;
            return targets;
        }

        @Override
        public boolean hasNotReached(double[] targets, DcMotor[] motors) {
            boolean hasNotReached = false;
            double currentPost = getAvg(new DcMotor[] {motors[1], motors[2]});
            double currentNeg = getAvg(new DcMotor[] {motors[0], motors[3]});
            if (targets[0] > currentPost || targets[1] < currentNeg) {
                hasNotReached = true;
            }
            return hasNotReached;
        }

        @Override
        public DcMotor[] getMotors() {
            return new DcMotor[]{topRight, topLeft, bottomRight, bottomLeft};
        }
    };
    /**
     * moves the motors to go to a specific direction
     * @param power
     */
    public abstract void setPower(double power);

    /**
     * finds the target tick count the motors should get to
     * @param distance
     * @param motors
     * @return
     */
    public abstract double[] getTarget(double distance, DcMotor[] motors);

    /**
     * determines whether or not the robot has reached the target
     * @param targets
     * @param motors
     * @return
     */
    public abstract boolean hasNotReached(double targets[], DcMotor[] motors);

    /**
     * gets the motors that are in use for the direction
     * @return
     */
    public abstract DcMotor[] getMotors();

    /**
     * stops the motors
     * @param motors
     */
    public static void stopRobot(DcMotor[] motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

}
