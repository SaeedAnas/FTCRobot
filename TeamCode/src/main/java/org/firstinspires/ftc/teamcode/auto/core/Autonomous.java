package org.firstinspires.ftc.teamcode.auto.core;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.auto.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.auto.vision.VisionThread;

import java.util.concurrent.Callable;
import java.util.concurrent.CompletionService;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

public abstract class Autonomous extends LinearOpMode {
    // contains all methods to move the robot

    // tune encoders becasue we are shit
    // measure
    // four mecanum wheels
    protected static DcMotor topRight;

    protected static DcMotor topLeft;

    protected static DcMotor bottomLeft;

    protected static DcMotor bottomRight;

    protected static DcMotor intakeLeft;

    protected static DcMotor intakeRight;

    protected static Servo foundationRight;

    protected static Servo foundationLeft;

    protected static Servo sideServo;

    protected static Servo grabber;

    protected static DcMotor cascadeRight;

    protected static DcMotor cascadeLeft;

    protected static Servo sideServoGrabber;

    protected static CRServo blockMover;

    private static Thread vision;

    private BNO055IMU imu;


    /**
     * sole method to initialize the robot -> call this function in the beginning of every opMode
     * initializes the imu, and sets the motors
     */
    public void initHardware() {
        startVision();
        initImu();
        topRight = hardwareMap.get(DcMotor.class, "frontRight");
        topLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "rearRight");
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        sideServo = hardwareMap.get(Servo.class, "sideServo");
        sideServoGrabber = hardwareMap.get(Servo.class, "sideServoGrabber");
        cascadeLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        cascadeRight = hardwareMap.get(DcMotor.class, "slideRight");
        grabber = hardwareMap.get(Servo.class, "grabber");
        blockMover = hardwareMap.get(CRServo.class, "blockMover");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        // armServo = hardwareMap.get(CRServo.class, "armServo");
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reverse right motors
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        cascadeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run using encoder
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    protected void printBlockPositon() {
        telemetry.addData("Block Positon: ", VisionPipeline.getBlockPosition());
        telemetry.update();
    }

    protected void startVision() {
        vision = new Thread(new VisionThread());
        vision.start();
    }
    protected void stopVision() {
        vision.interrupt();
    }

    public void calibrateMotors() {
        cascadeRight.setPower(0.5);
        sleep(1000);
        cascadeRight.setPower(-0.5);
        sleep(1000);
        cascadeRight.setPower(0);
        sleep(1000);
        cascadeLeft.setPower(0.5);
        sleep(1000);
        cascadeLeft.setPower(-0.5);
        sleep(1000);
        cascadeLeft.setPower(0);
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
        foundationLeft.setPosition(L_FOUNDATION_GRAB);
        foundationRight.setPosition(R_FOUNDATION_GRAB);
    }

    /**
     * brings the foundation servos up to release the foundation
     */
    protected void releaseFoundation() {
        foundationLeft.setPosition(L_FOUNDATON_RELEASE);
        foundationRight.setPosition(R_FOUNDATION_RELEASE);
    }

    protected void moveFoundation(double left, double right) {
        foundationLeft.setPosition(left);
        foundationRight.setPosition(right);
    }


//    private void autoArm() {
//        if (dpad1Down) {
//            sideServo.setPosition(0.6);
//            sleep(1000);
//            sideServoGrabber.setPosition(0.6);
//        } else if (dpad1Up) {
//            sideServo.setPosition(0.1);
//            sideServoGrabber.setPosition(0.2);
//        } else if (dpad1Left) {
//            sideServo.setPosition(0.1);
//            sideServoGrabber.setPosition(0.6);
//        }
//    }
    // TODO finish this method anto arm
    private void getBlock() {
        sideServo.setPosition();
        sideServoGrabber.setPosition();
    }
    protected void pickUpBlock() {

    }

    // TODO finish this method
    protected void dropBlock() {

    }

    // arm code

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

    /**
     * Moves the robot in the direction you specify
     * @param direction one of the Direction enums (ex. FORWARD, BACKWARD, LEFT)
     * @param distance distance you want to travel (100, 200), never less than 0
     * @param power power
     */
    protected void move(Direction direction, double distance, double power) {
            if (opModeIsActive()) {
                ExecutorService t = Executors.newFixedThreadPool(1);
                ExecutorService avg = Executors.newFixedThreadPool(1);
                CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
                CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);

                Future<Double> futureAvg;
                Future<Boolean> futureTarget;

                double degree = getGyroYAngle();
                double average;
                boolean hasNotReached;

                DcMotor[] motors = direction.getMotors();
                double target = direction.getTarget(distance, motors);

                Callable<Boolean> hNR = new targetGet(direction, motors, target);
                Callable<Double> avgCall = new average(motors);

                direction.setPower(power);

                futureTarget = targetService.submit(hNR);
                futureAvg = avgService.submit(avgCall);

                try {
                    hasNotReached = futureTarget.get();
                    average = futureAvg.get();

                    while (opModeIsActive() && hasNotReached) {

                        futureAvg = avgService.submit(avgCall);
                        futureTarget = targetService.submit(hNR);

                        direction.setPower(power);
                        telemetry.addData("Direction: ", direction);
                        telemetry.addData("Current: ", average);
                        telemetry.addData("Target: ", target);
                        telemetry.update();

                        hasNotReached = futureTarget.get();
                        average = futureAvg.get();
                    }

                } catch (Exception e) {
                    telemetry.addData("HI", "KENDALL IS GAY");
                    telemetry.update();
                }
                if (opModeIsActive()) {
                    Direction.stopRobot(motors);
                    if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
                        correctPosition(degree, 0.5);
                    }
                }
                Direction.stopRobot(motors);
                t.shutdownNow();
                avg.shutdownNow();
            }

    }

    protected void move(Strafe direction, double distance, double power){
            if (opModeIsActive()) {
                ExecutorService t = Executors.newFixedThreadPool(1);
                ExecutorService avg = Executors.newFixedThreadPool(1);
                CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
                CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);

                Future<Double> futureAvg;
                Future<Boolean> futureTarget;

                double degree = getGyroYAngle();
                double average;
                boolean hasNotReached;

                DcMotor[] motors = direction.getMotors();
                double target[] = direction.getTarget(distance, motors);

                Callable<Boolean> hNR = new targetGetS(direction, motors, target);
                Callable<Double> avgCall = new average(motors);

                direction.setPower(power);

                futureTarget = targetService.submit(hNR);
                futureAvg = avgService.submit(avgCall);

                try {

                    hasNotReached = futureTarget.get();
                    average = futureAvg.get();

                    while (opModeIsActive() && hasNotReached) {

                        futureAvg = avgService.submit(avgCall);
                        futureTarget = targetService.submit(hNR);

                        direction.setPower(power);
                        telemetry.addData("Direction: ", direction);
                        telemetry.addData("Current: ", average);
                        telemetry.addData("Target: ", target);
                        telemetry.update();

                        hasNotReached = futureTarget.get();
                        average = futureAvg.get();
                    }
                } catch (Exception e) {
                    telemetry.addData("HI", "KENDALL IS GAY");
                    telemetry.update();
                }

                if (opModeIsActive()) {

                    Strafe.stopRobot(motors);
                    if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
                        correctPosition(degree, 0.5);
                    }

                }

                Strafe.stopRobot(motors);
                t.shutdownNow();
                avg.shutdownNow();

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

    // Make has not reached and imu thread
    // telemetry

    class ImuGet implements Callable<Double> {

        @Override
        public Double call() throws Exception {
            return getGyroYAngle();
        }
    }

    class targetGet implements Callable<Boolean> {
        Direction direction;
        DcMotor[] motors;
        double target;
        targetGet(Direction direction, DcMotor[] motors, double target) {
            this.direction = direction;
            this.target = target;
            this.motors = motors;
        }

        @Override
        public Boolean call() throws Exception {
            return direction.hasNotReached(target,motors);
        }
    }

    class targetGetS implements Callable<Boolean> {
        Strafe direction;
        DcMotor[] motors;
        double[] target;
        targetGetS(Strafe direction, DcMotor[] motors, double[] target) {
            this.direction = direction;
            this.target = target;
            this.motors = motors;
        }

        @Override
        public Boolean call() throws Exception {
            return direction.hasNotReached(target,motors);
        }
    }

    class tel implements Runnable {

        @Override
        public void run() {

        }
    }

    class average implements Callable<Double> {
        DcMotor[] d;
        average(DcMotor[] d) {
            this.d = d;
        }

        @Override
        public Double call() {
            return getAvg(d);
        }
    }

    protected void autoCorrect(Direction direction, double distance, double power) throws Exception{
        if (opModeIsActive()) {
            ExecutorService t = Executors.newFixedThreadPool(1);
            ExecutorService avg = Executors.newFixedThreadPool(1);
            CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
            CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);

            Future<Double> futureAvg;
            Future<Boolean> futureTarget;
            double degree = getGyroYAngle();

            double average;
            boolean hasNotReached;

            DcMotor[] motors = direction.getMotors();
            double target = direction.getTarget(distance,motors);

            Callable<Boolean> hNR = new targetGet(direction, motors, target);
            Callable<Double> avgCall = new average(motors);

            direction.setPower(power);

            futureTarget = targetService.submit(hNR);
            futureAvg = avgService.submit(avgCall);

            hasNotReached = futureTarget.get();
            average = futureAvg.get();

            while(opModeIsActive() && hasNotReached) {

                futureAvg = avgService.submit(avgCall);
                futureTarget = targetService.submit(hNR);

                    direction.setPower(power);
                    telemetry.addData("Direction: ", direction);
                    telemetry.addData("Current: ", average);
                    telemetry.addData("Target: ", target);
                    telemetry.update();

                hasNotReached = futureTarget.get();
                average = futureAvg.get();
            }
            Direction.stopRobot(motors);
            if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
                correctPosition(degree, 0.5);
            }
            Direction.stopRobot(motors);
            t.shutdownNow();
            avg.shutdownNow();
        }
    }

    protected void autoCorrect(Strafe direction, double distance, double power) throws Exception{
        if (opModeIsActive()) {
            ExecutorService t = Executors.newFixedThreadPool(1);
            ExecutorService avg = Executors.newFixedThreadPool(1);
            CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
            CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);

            Future<Double> futureAvg;
            Future<Boolean> futureTarget;

            double degree = getGyroYAngle();
            double average;
            boolean hasNotReached;

            DcMotor[] motors = direction.getMotors();
            double target[] = direction.getTarget(distance,motors);

            Callable<Boolean> hNR = new targetGetS(direction, motors, target);
            Callable<Double> avgCall = new average(motors);

            direction.setPower(power);

            futureTarget = targetService.submit(hNR);
            futureAvg = avgService.submit(avgCall);

            hasNotReached = futureTarget.get();
            average = futureAvg.get();

            while(opModeIsActive() && hasNotReached) {

                futureAvg = avgService.submit(avgCall);
                futureTarget = targetService.submit(hNR);

                direction.setPower(power);
                telemetry.addData("Direction: ", direction);
                telemetry.addData("Current: ", average);
                telemetry.addData("Target: ", target);
                telemetry.update();

                hasNotReached = futureTarget.get();
                average = futureAvg.get();
            }

            Strafe.stopRobot(motors);
            if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
                correctPosition(degree, 0.3);
            }
            Strafe.stopRobot(motors);
            t.shutdownNow();
            avg.shutdownNow();
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
                targets[0] = getAvg(new DcMotor[] {motors[0], motors[3]}) + (COUNTS_PER_INCH * distance) + CORRECTION;
                targets[1] = getAvg(new DcMotor[] {motors[1], motors[2]}) - (COUNTS_PER_INCH * distance) - CORRECTION;
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
                return new DcMotor[]{topRight, topLeft, bottomRight, bottomLeft};
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
    public enum Direction {

        FORWARD {
            final double CORRECTION = 0 * COUNTS_PER_INCH;
            @Override
            public void setPower(double power) {
                topRight.setPower(power);
                topLeft.setPower(power);
                bottomRight.setPower(power);
                bottomLeft.setPower(power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
                double motorAvg = getAvg(motors);
                return motorAvg < target;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        BACKWARD {
            final double CORRECTION = 0 * COUNTS_PER_INCH;
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
            }

            @Override
            public boolean hasNotReached(double target, DcMotor[] motors) {
                return getAvg(motors) > target;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        FORWARD_LEFT {
            final double CORRECTION = 0 * COUNTS_PER_INCH;
            @Override
            public void setPower(double power) {
                topRight.setPower(power);
                bottomLeft.setPower(power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
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
            final double CORRECTION = 0 * COUNTS_PER_INCH;
            @Override
            public void setPower(double power) {
                topLeft.setPower(power);
                bottomRight.setPower(power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) + (COUNTS_PER_INCH * distance) + CORRECTION;
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
            final double CORRECTION = 0 * COUNTS_PER_INCH;
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
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
            final double CORRECTION = 0 * COUNTS_PER_INCH;

            @Override
            public void setPower(double power) {
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
            }

            @Override
            public double getTarget(double distance, DcMotor[] motors) {
                return getAvg(motors) - (COUNTS_PER_INCH * distance) - CORRECTION;
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

}