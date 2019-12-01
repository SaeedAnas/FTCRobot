package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "teleOp", group = "teloop")
public class Teleop extends LinearOpMode {
    //Hardware
    private DcMotor left, right, arm, arm2;
    private Servo rightFoundation, leftFoundation, grabber;
    private CRServo armServo;
    private DcMotor bottomLeft, topLeft, bottomRight, topRight;
    //Constants
    private static final double
            LEFT_FOUNDATION_DOWN = 1,
            LEFT_FOUNDATION_UP = 0,
            RIGHT_FOUNDATION_DOWN = 0,
            RIGHT_FOUNDATION_UP = 1,
            SERVO_GRAB = 0.3,
            SERVO_RELEASE = 0,
            FULL_SPEED = 1,
            ZERO_SPEED = 0,
            SLOW_DOWN1 = 1.3,
            SLOW_DOWN2 = 1.5;
    private BNO055IMU imu;
    private Orientation OriginAngle = new Orientation();
    private double deltaAngle;
    private int driver = 0;
    //boolean
    boolean gpad1x, gpad1y, gpad2a, gpad2b, gpad2rightBumper, gpad2leftBumper;

//    private void printStatus() {
//        telemetry.addData("rightservo current position", rightFoundation.getPosition());
//        telemetry.addData("leftservo current position", leftFoundation.getPosition());
//        //telemetry.addData("armservo current position", grabber.getPosition());
//        //telemetry.addData("y stick value:", gamepad1.right_stick_y);
//        //telemetry.addData("x stick value:", gamepad1.right_stick_y);
//        // telemetry.addData("y stick value:", gamepad1.right_stick_y);
//        telemetry.addData("x", gpad1x);
//        telemetry.update();
//
//
//    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    private void printStatus() {
        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initImu();
        //Init hardware
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        arm = hardwareMap.get(DcMotor.class, "armLeft");
        arm2 = hardwareMap.get(DcMotor.class, "armRight");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo = hardwareMap.get(CRServo.class, "armServo");


        //buttons g1xButton, g1yButton, g2aButton,g2bButton,g2rightBump,g2leftBump;
        gpad1x = gamepad1.x;
        gpad1y = gamepad1.y;
        gpad2a = gamepad2.a;
        gpad2b = gamepad2.b;
        gpad2rightBumper = gamepad2.right_bumper;
        gpad2leftBumper = gamepad2.left_bumper;

        // rightMotor is upside-down
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        //zero power behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for play
        waitForStart();
        while (opModeIsActive()) {
            printStatus();
            if (gamepad1.x) {

                rightFoundation.setPosition(0.6);
                leftFoundation.setPosition(0.4);

            }
            if (gamepad1.y) {
                rightFoundation.setPosition(0.1);
                leftFoundation.setPosition(0.9);

            }
            if (gamepad2.a) {
                grabber.setPosition(0.7);

            }
            if (gamepad2.b) {
                grabber.setPosition(0);
            }
            if (gamepad2.x) {
                armServo.setPower(0.7);

            } else if (gamepad2.y) {
                armServo.setPower(-0.7);
            } else {

                armServo.setPower(0);
            }
            if (gamepad2.right_trigger > 0.5) {
                arm.setPower(-1);
                arm2.setPower(-1);

            } else if (gamepad2.left_trigger > 0.5) {
                arm.setPower(1);
                arm2.setPower(1);
            } else {

                arm.setPower(0);
                arm2.setPower(0);
            }
            //arm();//Arm function

            move();//Drive function
        }

    }

    // Foundation code
    private void releaseFoundation() {
        rightFoundation.setPosition(RIGHT_FOUNDATION_UP);
        leftFoundation.setPosition(LEFT_FOUNDATION_UP);
    }

    private void grabFoundation() {
        rightFoundation.setPosition(RIGHT_FOUNDATION_DOWN);
        leftFoundation.setPosition(LEFT_FOUNDATION_DOWN);
    }

    //Grabber code
    private void servoGrab() {
        grabber.setPosition(SERVO_GRAB);
    }

    private void servoRelease() {
        grabber.setPosition(SERVO_RELEASE);
    }

    //Arm code
    private void arm() {
        if (gpad2rightBumper) {
            //armServo.setDirection(1);

        } else if (gpad2leftBumper) {
            //armServo.setDirection(-1);
        } else {

            armServo.setPower(ZERO_SPEED);
        }


        arm.setPower(gamepad2.left_trigger);
        arm.setPower(-gamepad2.right_trigger);
    }

    //Drive Code
//    private void drive(){
//        double leftPower1 = gamepad1.right_stick_y;
//        double rightPower1 = gamepad1.left_stick_y;
//        double leftPower2 = gamepad2.right_stick_y;
//        double rightPower2 = gamepad2.left_stick_y;
//
//        if(driver == 0) {
//            if(leftPower1 != 0 || rightPower1 != 0) {
//                right.setPower(rightPower1/SLOW_DOWN1);
//                left.setPower(leftPower1/SLOW_DOWN1);
//                driver = 1;
//            } else if (leftPower2 != 0 || rightPower2 != 0) {
//                right.setPower(rightPower2/SLOW_DOWN2);
//                left.setPower(leftPower2/ SLOW_DOWN2);
//                driver = 2;
//            }
//        } else if (driver == 1) {
//            if(leftPower1 != 0 || rightPower1 != 0) {
//                right.setPower(rightPower1/SLOW_DOWN1);
//                left.setPower(leftPower1/SLOW_DOWN1);
//            }
//            else {
//                driver = 0;
//                left.setPower(0);
//                right.setPower(0);
//            }
//        } else if (driver == 2){
//            if (leftPower2 != 0 || rightPower2 != 0) {
//                right.setPower(rightPower2/SLOW_DOWN2);
//                left.setPower(leftPower2/ SLOW_DOWN2);
//            } else {
//                driver = 0;
//                left.setPower(0);
//                right.setPower(0);
//            }
//        }
//
//    }
    private void move() {
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;
        double switchVal = gamepad1.left_trigger;

        if (leftY > 0) {
            forward(leftY);
        } else if (leftY < 0) {
            backward(-leftY);
        } else if (leftX > 0) {
            slideRight(leftX);
        } else if (leftX < 0) {
            slideLeft(-leftX);
        } else if (rightX > 0) {
            rotateRight(rightX);
        } else if (rightX < 0) {
            rotateLeft(-rightX);
        } else if (switchVal != 0) {
            if (rightY > 0) {
                strafeLeftFront(rightY);
            } else if (rightY < 0) {
                strafeLeftBack(-rightY);
            }
        } else if (rightY > 0) {
            strafeRightFront(rightY);
        } else if (rightY < 0) {
            strafeRightBack(-rightY);
        } else {
            stopRobot();
        }
    }


    private void stopRobot() {
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }


    private void forward(double power) {
        topRight.setPower(power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);
    }

    private void backward(double power) {
        topRight.setPower(-power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(-power);
    }

    private void slideLeft(double power) {
        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);
    }

    private void slideRight(double power) {
        topRight.setPower(-power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(-power);
    }

    private void rotateRight(double power) {
        topRight.setPower(-power);
        topLeft.setPower(power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);
    }

    private void rotateLeft(double power) {
        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);
    }

    private void strafeRightFront(double power) {
        topLeft.setPower(power);
        bottomRight.setPower(power);
    }

    private void strafeRightBack(double power) {
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
    }

    private void strafeLeftFront(double power) {
        topRight.setPower(power);
        bottomLeft.setPower(power);
    }

    private void strafeLeftBack(double power) {
        topRight.setPower(-power);
        bottomLeft.setPower(-power);
    }

}