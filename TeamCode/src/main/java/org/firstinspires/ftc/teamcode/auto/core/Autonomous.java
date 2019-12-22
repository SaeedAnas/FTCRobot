package org.firstinspires.ftc.teamcode.auto.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

public abstract class Autonomous extends LinearOpMode {
    // contains all methods to move the robot

    // four mecanum wheels
    private static DcMotor topRight;

    private static DcMotor topLeft;

    private static DcMotor bottomLeft;

    private static DcMotor bottomRight;

    // we don't have any of this yet

    private static DcMotor armMotorLeft;

    private static DcMotor armMotorRight;

    private static Servo grabber;

    private static Servo foundationRight;

    private static Servo foundationLeft;

    private static Servo sideServo;

    private BNO055IMU imu;

    static List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();

    // private static CRServo armServo;

    /**
     * sole method to initialize the robot -> call this function in the beginning of every opMode
     * initializes the imu, and sets the motors
     */
    public void initHardware() {
        //initVuforia();
        initImu();
        VariableManager.getInstance(telemetry);
//        leftMotor = hardwareMap.get(DcMotor.class, "left");
//        rightMotor = hardwareMap.get(DcMotor.class, "right");
//        armMotorLeft = hardwareMap.get(DcMotor.class, "armLeft");
//        armMotorRight = hardwareMap.get(DcMotor.class, "armRight");
//        foundationLeft = hardwareMap.get(Servo.class, "leftFoundation");
//        foundationRight = hardwareMap.get(Servo.class, "rightFoundation");
//        grabber = hardwareMap.get(Servo.class, "grabber");
        Thread vision = new Thread(new VisionThread());
        vision.run();
        topRight = hardwareMap.get(DcMotor.class, "frontRight");
        topLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "rearRight");
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        sideServo = hardwareMap.get(Servo.class, "sideServo");
        armMotorRight = hardwareMap.get(DcMotor.class, "slideRight");
        armMotorLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        // armServo = hardwareMap.get(CRServo.class, "armServo");
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reverse right motors
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightMotor is upside-down
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // reset the encoder
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run using encoder
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Ready!", "Press Start.");
        telemetry.update();
        // wait for play
        waitForStart();
    }

    /**
     * Initializes the imu -> may take a couple of seconds
     */
    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry.addData("Status: ", "Calibrating");
        telemetry.update();
        imu.initialize(parameters);

        telemetry.addData("Status: ", "Imu Calibration Ready");
        telemetry.update();

    }

    public void calibrateMotors() {
        armMotorRight.setPower(0.5);
        sleep(1000);
        armMotorRight.setPower(-0.5);
        sleep(1000);
        armMotorRight.setPower(0);
        sleep(1000);
        armMotorLeft.setPower(0.5);
        sleep(1000);
        armMotorLeft.setPower(-0.5);
        sleep(1000);
        armMotorLeft.setPower(0);
        sleep(1000);
    }

    /**
     * print the y value, which is the degree we use for imu turns
     */
    private void printDegree() {
        telemetry.addData("Y", getGyroYAngle());
    }

    /**
     * print the encoder values of the four wheels
     */
    private void print() {
        telemetry.addData("topLeft: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }


    protected void brake() {
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Foundation code

    /**
     * brings the foundation servos down to grab the foundation
     */
    protected void grabFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_DOWN);
        foundationRight.setPosition(RIGHT_FOUNDATION_DOWN);
    }

    /**
     * brings the foundation servos up to release the foundation
     */
    protected void releaseFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_UP);
        foundationRight.setPosition(RIGHT_FOUNDATION_UP);
    }

    protected void moveFoundation(double left, double right) {
        foundationLeft.setPosition(left);
        foundationRight.setPosition(right);
    }

    /**
     * uses the grab servo to hold on to a block
     */
    protected void grab() {
        grabber.setPosition(GRABBER_GRAB);
    }
    /**
     * uses the grab servo to release the block
     */
    protected void release() {
        grabber.setPosition(GRABBER_RELEASE);
    }

    // arm code

    /**
     * moves the arm up or down (-mm is down, +mm is up)
     * @param mm the distance in millimeters that you want to move (- is down, + is up)
     * @param power amount of power for the motors
     */
    protected void whileArm(double mm, double power) {
        double target = ((armMotorLeft.getCurrentPosition() + (mm * TICKS_PER_MM_ARM)) + (armMotorRight.getCurrentPosition() + (mm * TICKS_PER_MM_ARM))) / 2;
        if (mm < 0) {
            armMotorLeft.setPower(-power);
            armMotorRight.setPower(-power);
            while (opModeIsActive() && (armMotorLeft.getCurrentPosition() > target && armMotorRight.getCurrentPosition() > target)) {
                telemetry.addData("MovingArm", armMotorLeft.getCurrentPosition());
                telemetry.addData("TARGET", target);
                telemetry.update();
            }
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
        } else {
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
            while (opModeIsActive() && (armMotorLeft.getCurrentPosition() < target && armMotorRight.getCurrentPosition() < target)) {
                telemetry.addData("MovingArm", armMotorLeft.getCurrentPosition());
                telemetry.addData("TARGET", target);
                telemetry.update();
            }
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
        }
    }

// 179 -> -179
// y value
// right decreasing
// left increasing

    /**
     * gets the Y value from the imu
     * @return Y value of the imu
     */
    private double getGyroYAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return (angles.secondAngle);
    }

    /**
     *
     * @param zeroReference the angle the robot was at during the beginning of the turn
     * @param currentAngle the current angle of the robot
     * @return the difference between the currentAngle and zeroReference
     */
    private double adjustedAngle(double zeroReference, double currentAngle) {
        double adjusted = currentAngle - zeroReference;
        if (adjusted < -179) {
            adjusted += 360;
        } else if (adjusted > 180) {
            adjusted -= 360;
        }
        return adjusted;
    }

    /**
     * function to make it easier for the Red and Blue Auto modes to turn
     * @param motorPower power
     * @param targetDegree degree you want to turn (+ for right, - for left)
     */
    protected void turnByGyro(double motorPower, double targetDegree) {
        if (targetDegree < 0) {
            turnLeftByGyro(motorPower, -(targetDegree - CORRECTION));
        } else {
            turnRightByGyro(motorPower, (targetDegree - CORRECTION));
        }
    }

    /**
     * Turns the robot left
     * @param motorPower power
     * @param targetDegree degree you want to turn
     */
    private void turnLeftByGyro(double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
        topLeft.setPower(-motorPower);
        bottomLeft.setPower(-motorPower);
        topRight.setPower(motorPower);
        bottomRight.setPower(motorPower);
        while (opModeIsActive() && (angleTurned < targetDegree)) {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

    /**
     * turns the robot right
     * @param motorPower power
     * @param targetDegree degree you want to turn
     */
    private void turnRightByGyro(double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
        topLeft.setPower(motorPower);
        bottomLeft.setPower(motorPower);
        topRight.setPower(-motorPower);
        bottomRight.setPower(-motorPower);
        while (opModeIsActive() && (angleTurned > (-targetDegree))) {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

//    protected void move(Direction direction, double distance, double power) {
//        if (opModeIsActive()) {
//            DcMotor[] motors = direction.getMotors();
//            double[] targets = direction.getTarget(distance, motors);
//            direction.setPower(power);
//            while (opModeIsActive() && direction.hasNotReached(targets, motors)) {
//                    direction.setPower(power);
//                    telemetry.addData("Direction: ", direction);
//                    print();
//            }
//            Direction.stopRobot(motors);
//        }
//    }

    /**
     * Moves the robot in the direction you specify
     * @param direction one of the Direction enums (ex. FORWARD, BACKWARD, LEFT)
     * @param distance distance you want to travel (100, 200), never less than 0
     * @param power power
     */
    protected void move(Direction direction, double distance, double power) {
        if (opModeIsActive()) {
            // target
            DcMotor[] motors = direction.getMotors();
            double target = direction.getTarget(distance, motors);
            direction.setPower(power);
            while(opModeIsActive() && direction.hasNotReached(target, motors)) {
                telemetry.addData("Direction: ", direction);
                telemetry.addData("AvgCurrent", getAvg(motors));
                telemetry.addData("Target", target);
                telemetry.update();
            }
            Direction.stopRobot(motors);
        }
    }

    protected void move(Strafe direction, double distance, double power) {
        if (opModeIsActive()) {
            // target
            DcMotor[] motors = direction.getMotors();
            double[] target = direction.getTarget(distance, motors);
            direction.setPower(power);
            while(opModeIsActive() && direction.hasNotReached(target, motors)) {
                telemetry.addData("Direction: ", direction);
                telemetry.addData("AvgCurrent", getAvg(motors));
                telemetry.addData("Target", target);
                telemetry.update();
            }
            Direction.stopRobot(motors);
        }
    }

    /**
     * gets the average of all of the motors' current positions
     * @param motors array of DcMotors
     * @return average of the DcMotor current positions
     */
    protected static double getAvg(DcMotor[] motors) {
        double motorAvg = 0;
        for(DcMotor motor: motors) {
            motorAvg += motor.getCurrentPosition();
        }
        motorAvg/=motors.length;
        return motorAvg;
    }

    /**
     * Move function that auto corrects when it is moved off track
     * @param direction one of the Direction enums (FORWARD, BACKWARD, LEFT)
     * @param distance distance you want to travel
     * @param power power
     */
    protected void autoCorrectMove(Direction direction, double distance, double power) {
        if (opModeIsActive()) {
            double degree = getGyroYAngle();
            double currentDegree;
            DcMotor[] motors = direction.getMotors();
            double target = direction.getTarget(distance, motors);
            direction.setPower(power);
            while (opModeIsActive() && direction.hasNotReached(target, motors)) {
                currentDegree = getGyroYAngle();
                if ((currentDegree < degree - DEGREE_THRESHOLD) || currentDegree > degree + DEGREE_THRESHOLD) {
                    correctPosition(degree, power);
                }
                else {
                    direction.setPower(power);
                    telemetry.addData("Degree", degree);
                    telemetry.addData("CurrentDegree", currentDegree);
                    telemetry.addData("Direction: ", direction);
                    telemetry.update();
                }
            }

            Direction.stopRobot(motors);
        }
    }

    protected void autoCorrectMove(Strafe direction, double distance, double power) {
        if (opModeIsActive()) {
            double degree = getGyroYAngle();
            double currentDegree;
            DcMotor[] motors = direction.getMotors();
            double[] target = direction.getTarget(distance, motors);
            direction.setPower(power);
            while (opModeIsActive() && direction.hasNotReached(target, motors)) {
                currentDegree = getGyroYAngle();
                if ((currentDegree < degree - DEGREE_THRESHOLD) || currentDegree > degree + DEGREE_THRESHOLD) {
                    correctPosition(degree, power);
                }
                else {
                    direction.setPower(power);
                    telemetry.addData("Degree", degree);
                    telemetry.addData("CurrentDegree", currentDegree);
                    telemetry.addData("Direction: ", direction);
                    telemetry.update();
                }
            }

            Direction.stopRobot(motors);
        }
    }

    /**
     * Moves the robot to the specified degree (different from turns)
     * @param degree degree you want to go back to
     * @param power power
     */
    private void correctPosition(double degree, double power) {
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (isRight(degree)) {
            topLeft.setPower(power);
            bottomLeft.setPower(power);
            topRight.setPower(-power);
            bottomRight.setPower(-power);
        } else {
            topLeft.setPower(-power);
            bottomLeft.setPower(-power);
            topRight.setPower(power);
            bottomRight.setPower(power);
        }
        double currentAngle = getGyroYAngle();
        while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) && (currentAngle > degree + DEGREE_THRESHOLD))) {
            telemetry.addData("Status: ", "Correcting");
            telemetry.update();
            currentAngle = getGyroYAngle();
        }

        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * determines whether turning right or left is the fasted way to get to a certain degree
     * @param targetDegree degree you want to get back to
     * @return
     */
    private boolean isRight(double targetDegree) {
        double currentAngle = getGyroYAngle();
        boolean isRight = false;
        // left increases
        // right decreases
        // -179 -> 179
        // if the current angle is negative and the degree > 0, then the fastest way to get to the degree is to go the opposite way.
        // degree = 160-360 , -160
        // current-target < 0 go left
        if ((currentAngle < 0 && targetDegree > 0)) {
            if (currentAngle < -90 && targetDegree > 90) {
                isRight = true;
            } else if (currentAngle > -90 && targetDegree < 90){
                isRight = false;
            }
        } else if ((currentAngle > 0 && targetDegree < 0)) {
            if (currentAngle > 90 && targetDegree < -90) {
                isRight = false;
            } else if (currentAngle < 90 && targetDegree > -90) {
                isRight = true;
            }
        }
        else if (currentAngle < targetDegree) {
            isRight = false;
        }
        else if (currentAngle > targetDegree) {
            isRight = true;
        }
        return isRight;
    }

    //right(negitive) currentAngle is negitive. currentAngle + Math.abs(currentAngle)
    //left(positve) currentAngle is positive. currentAngle - Math.abs(currentAngle)
    /**
     * an enum for the move function
     * has four abstract methods
     */

    public enum Strafe {
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
                double[] targets = new double[2];
                targets[0] = getAvg(new DcMotor[] {motors[0], motors[3]}) + (COUNTS_PER_INCH * distance);
                targets[1] = getAvg(new DcMotor[] {motors[1], motors[2]}) - (COUNTS_PER_INCH * distance);
                return targets;
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                boolean hasNotReached = false;
                double currentPos = getAvg(new DcMotor[] {motors[0], motors[3]});
                double currentNeg = getAvg(new DcMotor[] {motors[1], motors[2]});
                if (targets[0] > currentPos || targets[1] < currentNeg) {
                    hasNotReached = true;
                }
                return hasNotReached;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
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
                double[] targets = new double[2];
                targets[0] = getAvg(new DcMotor[] {motors[1], motors[2]}) + (COUNTS_PER_INCH * distance);
                targets[1] = getAvg(new DcMotor[] {motors[0], motors[3]}) - (COUNTS_PER_INCH * distance);
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
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
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
            public double getTarget(double distance, DcMotor[] motors) {
//                double[] targets = new double[motors.length];
//                for (int i = 0; i < targets.length; i++) {
//                    targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
//                }
//                return targets;
                return getAvg(motors) + (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
//                boolean hasNotReached = true;
//                for (int i = 0; i < targets.length; i++) {
//                    if (motors[i].getCurrentPosition() > targets[i]) {
//                        hasNotReached = false;
//                        break;
//                    }
//                }
//                return hasNotReached;
                double motorAvg = getAvg(motors);
                return motorAvg < target;
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
            public double getTarget(double distance, DcMotor[] motors) {
//                double[] targets = new double[motors.length];
//                for (int i = 0; i < targets.length; i++) {
//                    targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
//                }
//                return targets;
                return getAvg(motors) - (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
//                boolean hasNotReached = true;
////                for (int i = 0; i < targets.length; i++) {
////                    if (motors[i].getCurrentPosition() < targets[i]) {
////                        hasNotReached = false;
////                        break;
////                    }
////                }
////                return hasNotReached;
                double motorAvg = getAvg(motors);
                return motorAvg > target;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
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
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) + (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
                double motorAvg = getAvg(motors);
                return motorAvg < target;
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
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) + (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
                double motorAvg = getAvg(motors);
                return motorAvg < target;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        BACKWARD_LEFT {
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) - (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double targets, DcMotor[] motors) {
                double motorAvg = getAvg(motors);
                return motorAvg > targets;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, bottomLeft};
            }
        },
        // TODO Change the getTarget and hasNotReached
        BACKWARD_RIGHT {
            @Override
            public void setPower(double power) {
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) - (COUNTS_PER_INCH * distance);
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
                double motorAvg = getAvg(motors);
                return motorAvg > target;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topLeft, bottomRight};
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
        public abstract double getTarget(double distance, DcMotor[] motors);

        /**
         * determines whether or not the robot has reached the target
         * @param targets
         * @param motors
         * @return
         */
        public abstract boolean hasNotReached(double targets, DcMotor[] motors);

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

    /**
     * calibrates the robot wheels
     */
    protected void calibrate() {
        topLeft.setPower(0.5);
        sleep(1000);
        print();
        topLeft.setPower(-0.5);
        sleep(1000);
        print();
        topRight.setPower(0.5);
        sleep(1000);
        print();
        topRight.setPower(-0.5);
        sleep(1000);
        print();
        bottomLeft.setPower(0.5);
        sleep(1000);
        print();
        bottomLeft.setPower(-0.5);
        sleep(1000);
        print();
        bottomRight.setPower(0.5);
        sleep(1000);
        print();
        bottomRight.setPower(-0.5);
        sleep(1000);
        print();
//        rotateMotorsOnce(FORWARD, 1);
//        sleep(1000);
//        rotateMotorsOnce(BACKWARD, 1);
//        sleep(1000);
//        rotateMotorsOnce(LEFT, 1);
//        sleep(1000);
//        rotateMotorsOnce(RIGHT, 1);
//        sleep(1000);
//        rotateMotorsOnce(FORWARD_LEFT, 1);
//        sleep(1000);
//        rotateMotorsOnce(BACKWARD_LEFT, 1);
//        sleep(1000);
//        rotateMotorsOnce(FORWARD_RIGHT, 1);
//        sleep(1000);
//        rotateMotorsOnce(BACKWARD_RIGHT, 1);
//        sleep(1000);

    }

    /**
     * Moves the robot in a direction for 1 second
     * @param direction a Direction enum to specify which direction you want to travel
     * @param power power
     */
    protected void rotateMotorsOnce(Direction direction, double power) {
        DcMotor[] motors = direction.getMotors();
        direction.setPower(power);
        sleep(1000);
        Direction.stopRobot(motors);
    }




    protected void regularDemonstration() {
        for(long i = 0; i < 10000000; i++) {
            telemetry.addData("Count: ", i);
            telemetry.update();
        }
        for (long i = 10000000; i > 0; i--) {
            telemetry.addData("Count: ", i);
            telemetry.update();
        }
    }

    public boolean isActive() {
        return opModeIsActive();
    }
}