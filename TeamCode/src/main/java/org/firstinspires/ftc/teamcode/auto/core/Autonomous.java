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
import org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric.Imu;
import org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric.ImuService;

import java.util.Arrays;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletionService;
import java.util.concurrent.ExecutorCompletionService;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

public abstract class Autonomous extends LinearOpMode {
    // contains all methods to correctMove the robot

    // tune encoders becasue we are shit
    // measure
    // four mecanum wheels
    public static DcMotor topRight;

    public static DcMotor topLeft;

    public static DcMotor bottomLeft;

    public static DcMotor bottomRight;

    protected static DcMotor intakeLeft;

    protected static DcMotor intakeRight;

    protected static Servo foundationRight;

    protected static Servo foundationLeft;

    protected static Servo sideServo;

    protected static Servo grabber;

    protected static DcMotor cascadeRight;

    protected static DcMotor cascadeLeft;

    protected static Servo sideServoGrabber;

    protected static Servo capStone;

    protected static CRServo blockMover;

    private static Thread vision;

    private BNO055IMU imu;

    public ImuService imuService;

    public Thread serviceThread;


    /**
     * sole method to initialize the robot -> call this function in the beginning of every opMode
     * initializes the imu, and sets the motors
     */
    public void initHardware() {
        startVision();
            initImu();
            imuService = new ImuService(imu);

        serviceThread = new Thread(imuService);
        serviceThread.start();
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
        capStone = hardwareMap.get(Servo.class, "capStone");
        blockMover.setDirection(DcMotorSimple.Direction.REVERSE);
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
        Imu.getInstance(imu);

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
    protected void stopProcesses() {
        vision.interrupt();
        serviceThread.interrupt();
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

    protected void moveServo(double power) {
        blockMover.setPower(power);
        sleep(2000);
        blockMover.setPower(0);
    }

    protected void encoderSlides(double power, double distance) {
      //  double target =
    }


    // Foundation code


    //
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

    protected void scanMode() {
        capStone.setPosition(CAP_SCAN);
    }
    protected void off() {
        capStone.setPosition(CAP_OFF);
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

    protected void moveCascades(double direction) {
        if (direction > 0) {
            cascadeLeft.setPower(0.8);
            cascadeRight.setPower(0.8);
            sleep(600);
        } else {
            cascadeLeft.setPower(-0.8);
            cascadeRight.setPower(-0.8);
            sleep(700);
        }
        cascadeLeft.setPower(0);
        cascadeRight.setPower(0);
    }

    protected void pickUpBlock() {
        grabber.setPosition(AUTO_GRAB_DOWN);


    }

    protected void dropBlock() {
        grabber.setPosition(AUTO_GRAB_UP);
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
        double adjusted = currentAngle + zeroReference;
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
            turnLeftByGyro(motorPower, (targetDegree+CORRECTION));
        } else {
            turnRightByGyro(motorPower, (targetDegree-CORRECTION));
        }
    }

    /**
     * Turns the robot left
     * @param motorPower power
     * @param targetDegree degree you want to turn
     */
    private void turnLeftByGyro(double motorPower, double targetDegree) {
        imuService.resetService();
        double actualAngle; // -50
        double angleTurned = imuService.getCurrentAngle(); // 0
        topLeft.setPower(-motorPower);
        bottomLeft.setPower(-motorPower);
        topRight.setPower(motorPower);
        bottomRight.setPower(motorPower);

        while (opModeIsActive() && (angleTurned > targetDegree)) {
            actualAngle = imuService.getActualAngle();
            angleTurned = imuService.getCurrentAngle();
            telemetry.addData("currentAngle", actualAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
    /**
     * turns the robot right
     * @param motorPower power
     * @param targetDegree degree you want to turn
     */
    private void turnRightByGyro(double motorPower, double targetDegree) {
        imuService.resetService();
        double actualAngle; // -50
        double angleTurned = imuService.getCurrentAngle(); // 0
        topLeft.setPower(motorPower);
        bottomLeft.setPower(motorPower);
        topRight.setPower(-motorPower);
        bottomRight.setPower(-motorPower);

        while (opModeIsActive() && (angleTurned < targetDegree)) {
            actualAngle = imuService.getActualAngle();
            angleTurned = imuService.getCurrentAngle();
            telemetry.addData("currentAngle", actualAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }

    public void turn(double power, double targetDegree) {
        imuService.resetService();

        double currentAngle = imuService.getCurrentAngle();
        double degree = adjustedAngle(targetDegree, currentAngle);

        if (targetDegree > 0) {
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

        while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) || (currentAngle > degree + DEGREE_THRESHOLD))) {
            telemetry.addData("Status: ", "Turning");
            telemetry.addData("startingAngle", degree);
            telemetry.addData("currentDegree", currentAngle);
            telemetry.update();
            currentAngle = imuService.getCurrentAngle();
        }

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);

        telemetry.addData("Status: ", "Turned");
        telemetry.addData("startingAngle", degree);
        telemetry.addData("currentDegree", currentAngle);
        telemetry.update();
    }


    /**
     * Moves the robot in the direction you specify
     * @param direction one of the Direction enums (ex. FORWARD, BACKWARD, LEFT)
     * @param distance distance you want to travel (100, 200), never less than 0
     * @param power power
     */

    // we can make the motors ramp up as well as slow down as it gets close to the target under a certain threshold
    /* Better AutoCorrect
    * Basically we swerve back into position
    * 
    * */
    protected void correctMove(Direction direction, double distance, double power) {
            if (opModeIsActive()) {
                ExecutorService t = Executors.newFixedThreadPool(1);
                ExecutorService current = Executors.newFixedThreadPool(1);
                CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
                CompletionService<double[]> currentService = new ExecutorCompletionService<>(current);

                Future<Boolean> futureTarget;
                Future<double[]> futureCurrents;

                double startingAngle = imuService.getActualAngle();
                boolean hasNotReached;
                double[] currents;

                DcMotor[] motors = direction.getMotors();
                double[] target = direction.getTarget(distance, motors);
              //  double[] start;
                String targetString = Arrays.toString(target);

                Callable<Boolean> hNR = new targetGet(direction, motors, target);
                Callable<double[]> cur = new current(motors);

                direction.setPower(power);

                futureTarget = targetService.submit(hNR);
                futureCurrents = currentService.submit(cur);



                try {
                    hasNotReached = futureTarget.get();
                    currents = futureCurrents.get();
                 //   start = currents;
                  //  double[] targetDiffernece = arraySubtract(target, start);

                    while (opModeIsActive() && hasNotReached) {

                        futureTarget = targetService.submit(hNR);
                        futureCurrents = currentService.submit(cur);
                       direction.setPower(power);
                        //direction.calcPower(targetDiffernece , arraySubtract(currents, start), power);
                        telemetry.addData("Direction: ", direction);
                        telemetry.addData("Current: ", Arrays.toString(currents));
                        telemetry.addData("Target: ", targetString);
                        telemetry.update();

                        hasNotReached = futureTarget.get();
                        currents = futureCurrents.get();
                    }

                } catch (Exception e) {
                    telemetry.addData("HI", "Anas IS GAY");
                    telemetry.update();
                }
                Direction.stopRobot(motors);
                t.shutdownNow();
                current.shutdownNow();
                correctPosition(startingAngle, 0.2);
            }
    }

    protected void move(Direction direction, double distance, double power) {
        if (opModeIsActive()) {
            ExecutorService t = Executors.newFixedThreadPool(1);
            ExecutorService current = Executors.newFixedThreadPool(1);
            CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
            CompletionService<double[]> currentService = new ExecutorCompletionService<>(current);

            Future<Boolean> futureTarget;
            Future<double[]> futureCurrents;

            double startingAngle = imuService.getActualAngle();
            boolean hasNotReached;
            double[] currents;

            DcMotor[] motors = direction.getMotors();
            double[] target = direction.getTarget(distance, motors);
            //  double[] start;
            String targetString = Arrays.toString(target);

            Callable<Boolean> hNR = new targetGet(direction, motors, target);
            Callable<double[]> cur = new current(motors);

            direction.setPower(power);

            futureTarget = targetService.submit(hNR);
            futureCurrents = currentService.submit(cur);



            try {
                hasNotReached = futureTarget.get();
                currents = futureCurrents.get();
                //   start = currents;
                //  double[] targetDiffernece = arraySubtract(target, start);

                while (opModeIsActive() && hasNotReached) {

                    futureTarget = targetService.submit(hNR);
                    futureCurrents = currentService.submit(cur);
                    direction.setPower(power);
                    //direction.calcPower(targetDiffernece , arraySubtract(currents, start), power);
                    telemetry.addData("Direction: ", direction);
                    telemetry.addData("Current: ", Arrays.toString(currents));
                    telemetry.addData("Target: ", targetString);
                    telemetry.update();

                    hasNotReached = futureTarget.get();
                    currents = futureCurrents.get();
                }

            } catch (Exception e) {
                telemetry.addData("HI", "Anas IS GAY");
                telemetry.update();
            }
            Direction.stopRobot(motors);
            t.shutdownNow();
            current.shutdownNow();
        }
    }




    /*
    motor.setPower(
     */


//    protected void correctMove(Slide direction, double distance, double power){
//            if (opModeIsActive()) {
//                ExecutorService t = Executors.newFixedThreadPool(1);
//                ExecutorService avg = Executors.newFixedThreadPool(1);
//                CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
//                CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);
//
//                Future<Double> futureAvg;
//                Future<Boolean> futureTarget;
//
//                double degree = getGyroYAngle();
//                double average;
//                boolean hasNotReached;
//
//                DcMotor[] motors = direction.getMotors();
//                double target[] = direction.getTarget(distance, motors);
//
//                Callable<Boolean> hNR = new targetGetS(direction, motors, target);
//                Callable<Double> avgCall = new averageS(motors);
//
//                direction.setPower(power);
//
//                futureTarget = targetService.submit(hNR);
//                futureAvg = avgService.submit(avgCall);
//
//                double real = target[0] - getAvg(new DcMotor[] {direction.getMotors()[1], direction.getMotors()[2]});
//                double current = getAvg(new DcMotor[] {direction.getMotors()[1], direction.getMotors()[2]});
//                double revUp = target[0] - (real * 0.9);
//                double slowDown = target[0] - (real * 0.1);
//
//                double firstEquation = current/distance;
//                double secondEquation = distance/current;
//                double finalEquation = firstEquation - secondEquation;
//                double firstDistance = (distance*4)/6;
//                try {
//
//                    hasNotReached = futureTarget.get();
//                    average = futureAvg.get();
//
//                    while (opModeIsActive() && hasNotReached) {
//
//                        if (current < firstDistance) {
//                            direction.setPower(1);
////                            direction.setPower(average/revUp);
//
//                        } else if (current > firstDistance ) {
//                            direction.setPower(finalEquation);
////                            direction.setPower(1 - (average/target[0]));
//                        }
////                        else {
////                            direction.setPower(power);
////                        }
//
//                        futureAvg = avgService.submit(avgCall);
//                        futureTarget = targetService.submit(hNR);
//
//                        direction.setPower(power);
//                        telemetry.addData("Direction: ", direction);
//                        telemetry.addData("Current: ", average);
//                        telemetry.addData("Target: ", target);
//                        telemetry.update();
//
//                        hasNotReached = futureTarget.get();
//                        average = futureAvg.get();
//                    }
//                } catch (Exception e) {
//                    telemetry.addData("HI", "KENDALL IS GAY");
//                    telemetry.update();
//                }
//
//                if (opModeIsActive()) {
//
//                    Slide.stopRobot(motors);
//                    if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
//                        correctPosition(degree, 0.5);
//                    }
//
//                }
//
//                Slide.stopRobot(motors);
//                t.shutdownNow();
//                avg.shutdownNow();
//
//            }
//    }


//    protected void autoCorrectMove(Direction direction, double distance, double power) {
//        if (opModeIsActive()) {
//            double degree = getGyroYAngle();
//            double currentDegree;
//            DcMotor[] motors = direction.getMotors();
//            double target = direction.getTarget(distance, motors);
//            direction.setPower(power);
//            while (opModeIsActive() && direction.hasNotReached(target, motors)) {
//                currentDegree = getGyroYAngle();
//                if ((currentDegree < degree - DEGREE_THRESHOLD) || currentDegree > degree + DEGREE_THRESHOLD) {
//                    correctPosition(degree, power);
//                }
//                else {
//                    direction.setPower(power);
//                    telemetry.addData("Degree", degree);
//                    telemetry.addData("CurrentDegree", currentDegree);
//                    telemetry.addData("Direction: ", direction);
//                    telemetry.update();
//                }
//            }
//
//            Direction.stopRobot(motors);
//        }
//    }

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
        double[] target;
        targetGet(Direction direction, DcMotor[] motors, double[] target) {
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
        Slide direction;
        DcMotor[] motors;
        double[] target;
        targetGetS(Slide direction, DcMotor[] motors, double[] target) {
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

    class current implements Callable<double[]> {
        DcMotor[] d;
        current(DcMotor[] d) {
            this.d = d;
        }

        @Override
        public double[] call() {
            return getCurrent(d);
        }
    }

    class averageS implements Callable<Double> {
        DcMotor[] d;
        averageS(DcMotor[] d) {
            this.d = d;
        }

        @Override
        public Double call() {
            return Direction.getAvg(new DcMotor[] {d[1], d[2]});
        }
    }

    private double[] getCurrent(DcMotor[] motors) {
        double[] currents = new double[motors.length];
        for(int i = 0; i < motors.length; i++) {
            currents[i] = motors[i].getCurrentPosition();
        }
        return currents;
    }

//    protected void autoCorrect(Direction direction, double distance, double power) throws Exception{
//        if (opModeIsActive()) {
//            ExecutorService t = Executors.newFixedThreadPool(1);
//            ExecutorService avg = Executors.newFixedThreadPool(1);
//            CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
//            CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);
//
//            Future<Double> futureAvg;
//            Future<Boolean> futureTarget;
//            double degree = getGyroYAngle();
//
//            double average;
//            boolean hasNotReached;
//
//            DcMotor[] motors = direction.getMotors();
//            double target = direction.getTarget(distance,motors);
//
//            Callable<Boolean> hNR = new targetGet(direction, motors, target);
//            Callable<Double> avgCall = new average(motors);
//
//            direction.setPower(power);
//
//            futureTarget = targetService.submit(hNR);
//            futureAvg = avgService.submit(avgCall);
//
//            hasNotReached = futureTarget.get();
//            average = futureAvg.get();
//
//            while(opModeIsActive() && hasNotReached) {
//
//                futureAvg = avgService.submit(avgCall);
//                futureTarget = targetService.submit(hNR);
//
//                    direction.setPower(power);
//                    telemetry.addData("Direction: ", direction);
//                    telemetry.addData("Current: ", average);
//                    telemetry.addData("Target: ", target);
//                    telemetry.update();
//
//                hasNotReached = futureTarget.get();
//                average = futureAvg.get();
//            }
//            Direction.stopRobot(motors);
//            if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
//                correctPosition(degree, 0.5);
//            }
//            Direction.stopRobot(motors);
//            t.shutdownNow();
//            avg.shutdownNow();
//        }
//    }

//    protected void autoCorrect(Slide direction, double distance, double power) throws Exception{
//        if (opModeIsActive()) {
//            ExecutorService t = Executors.newFixedThreadPool(1);
//            ExecutorService avg = Executors.newFixedThreadPool(1);
//            CompletionService<Boolean> targetService = new ExecutorCompletionService<>(t);
//            CompletionService<Double> avgService = new ExecutorCompletionService<>(avg);
//
//            Future<Double> futureAvg;
//            Future<Boolean> futureTarget;
//
//            double degree = getGyroYAngle();
//            double average;
//            boolean hasNotReached;
//
//            DcMotor[] motors = direction.getMotors();
//            double target[] = direction.getTarget(distance,motors);
//
//            Callable<Boolean> hNR = new targetGetS(direction, motors, target);
//            Callable<Double> avgCall = new average(motors);
//
//            direction.setPower(power);
//
//            futureTarget = targetService.submit(hNR);
//            futureAvg = avgService.submit(avgCall);
//
//            hasNotReached = futureTarget.get();
//            average = futureAvg.get();
//
//            while(opModeIsActive() && hasNotReached) {
//
//                futureAvg = avgService.submit(avgCall);
//                futureTarget = targetService.submit(hNR);
//
//                direction.setPower(power);
//                telemetry.addData("Direction: ", direction);
//                telemetry.addData("Current: ", average);
//                telemetry.addData("Target: ", target);
//                telemetry.update();
//
//                hasNotReached = futureTarget.get();
//                average = futureAvg.get();
//            }
//
//            Slide.stopRobot(motors);
//            if (getGyroYAngle() < degree - DEGREE_THRESHOLD || getGyroYAngle() > degree + DEGREE_THRESHOLD) {
//                correctPosition(degree, 0.3);
//            }
//            Slide.stopRobot(motors);
//            t.shutdownNow();
//            avg.shutdownNow();
//        }
//    }
    /**
     * Moves the robot to the specified degree (different from turns)
     * @param degree degree you want to go back to
     * @param power power
     */
    public void correctPosition(double degree, double power) {

        double currentAngle = imuService.getCurrentAngle();


        if (isRight(degree)) {
            telemetry.addData("startingAngle", degree);
            telemetry.addData("currentDegree", currentAngle);
            telemetry.addData("Is", "RIGHT");
            telemetry.update();

            topLeft.setPower(power);
            bottomLeft.setPower(power);
            topRight.setPower(-power);
            bottomRight.setPower(-power);

        } else {
            telemetry.addData("startingAngle", degree);
            telemetry.addData("currentDegree", currentAngle);
            telemetry.addData("Is", "LEFT");
            telemetry.update();

            topLeft.setPower(-power);
            bottomLeft.setPower(-power);
            topRight.setPower(power);
            bottomRight.setPower(power);
        }


        while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) || (currentAngle > degree + DEGREE_THRESHOLD))) {
            telemetry.addData("Status: ", "Correcting");
            telemetry.addData("startingAngle", degree);
            telemetry.addData("currentDegree", currentAngle);
            telemetry.update();
            currentAngle = imuService.getCurrentAngle();
        }

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);

        telemetry.addData("Status: ", "Corrected");
        telemetry.addData("startingAngle", degree);
        telemetry.addData("currentDegree", currentAngle);
        telemetry.update();
    }

    /**
     * determines whether turning right or left is the fasted way to get to a certain degree
     * @param targetDegree degree you want to get back to
     * @return
     */
    private boolean isRight(double targetDegree) {
        double currentAngle = imuService.getCurrentAngle();
        boolean isRight = false;

        // right increases
        // left decreases
        // -179 -> 179
        // if the current angle is negative and the degree > 0, then the fastest way to get to the degree is to go the opposite way.
        // degree = 160-360 , -160
        // current-target < 0 go left

        if ((currentAngle < 0 && targetDegree > 0)) {
            if (currentAngle < -90 && targetDegree > 90) {
                isRight = false;
            } else if (currentAngle > -90 && targetDegree < 90){
                isRight = true;
            }
        } else if ((currentAngle > 0 && targetDegree < 0)) {
            if (currentAngle > 90 && targetDegree < -90) {
                isRight = true;
            } else if (currentAngle < 90 && targetDegree > -90) {
                isRight = false;
            }
        }

        else if (currentAngle < targetDegree) {
            isRight = true;
        }

        else if (currentAngle > targetDegree) {
            isRight = false;
        }

        return isRight;

    }

    //right(negitive) currentAngle is negitive. currentAngle + Math.abs(currentAngle)
    //left(positve) currentAngle is positive. currentAngle - Math.abs(currentAngle)
    /**
     * an enum for the correctMove function
     * has four abstract methods
     */

    protected void stopAllMotors() {
        topRight.setPower(0);topLeft.setPower(0);bottomLeft.setPower(0);bottomRight.setPower(0);cascadeRight.setPower(0);cascadeLeft.setPower(0);blockMover.setPower(0);
    }

    private static double round (double value, int precision) {
        int scale = (int) Math.pow(10, precision);
        return (double) Math.round(value * scale) / scale;
    }

    private static double[] arraySubtract(double[] arr, double[] subtractArr) {
        if (arr.length != subtractArr.length) {
            return arr;
        }
        double[] newArr = new double[arr.length];
        for(int i = 0; i < arr.length; i++) {
            newArr[i]  = arr[i] - subtractArr[i];
        }
        return newArr;
    }
}