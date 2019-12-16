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

@TeleOp(name = "tele", group = "teloop")
public class Tele extends LinearOpMode {

    // Hardware
    private static DcMotor topRight;

    private static DcMotor topLeft;

    private static DcMotor bottomLeft;

    private static DcMotor bottomRight;

    private static Servo foundationRight;

    private static Servo foundationLeft;

    private static Servo sideServo;


    // Constants
    private static final double
            LEFT_FOUNDATION_DOWN = 1,
            LEFT_FOUNDATION_UP = 0,
            RIGHT_FOUNDATION_DOWN = 0,
            RIGHT_FOUNDATION_UP = 1,
            SERVO_GRAB = 0.0,
            SERVO_RELEASE = 0.7,
            SLOW_DOWN1 = 1.3,
            SLOW_DOWN2 = 1.5;
    private static boolean isUp = false;
    private static boolean gpad1x, gpad1y, gpad2a, gpad2b, gpad2x, gpad2y, gpad2rightBumper, gpad2leftBumper, gpad1rightBumper, gpad1leftBumper;
    private static double leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, gpad1leftTrigger, gpad1rightTrigger, gpad2leftTrigger, gpad2rightTrigger;

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        topRight = hardwareMap.get(DcMotor.class, "frontRight");
        topLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "rearRight");
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        sideServo = hardwareMap.get(Servo.class, "sideServo");
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

        waitForStart();
        while (opModeIsActive()) {
            getValues();
            foundation();
            move();
            printValues();
        }
    }

    private void getValues() {
        gpad1x = gamepad1.x;
        gpad1y = gamepad1.y;
        gpad2a = gamepad2.a;
        gpad2b = gamepad1.b;
        gpad2x = gamepad2.x;
        gpad2y = gamepad2.y;
        gpad1leftTrigger = gamepad1.left_trigger;
        gpad1rightTrigger = gamepad1.right_trigger;
        gpad2leftTrigger = gamepad2.left_trigger;
        gpad2rightTrigger = gamepad2.right_trigger;
        gpad1leftBumper = gamepad1.left_bumper;
        gpad1rightBumper = gamepad1.right_bumper;
        gpad2rightBumper = gamepad2.right_bumper;
        gpad2leftBumper = gamepad2.left_bumper;
        leftX1 = gamepad1.left_stick_x;
        leftY1 = gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        rightY1 = gamepad1.right_stick_y;
        leftX2 = gamepad2.left_stick_x;
        leftY2 = gamepad2.left_stick_y;
        rightX2 = gamepad2.right_stick_x;
        rightY2 = gamepad2.right_stick_y;
    }

    private void printValues() {
        telemetry.addData("leftX1", leftX1);
        telemetry.addData("leftY1",leftY1);
        telemetry.addData("rightX1",rightX1);
        telemetry.addData("rightY1",rightY1);
        telemetry.addData("leftX2",leftX2);
        telemetry.addData("leftY2",leftY2);
        telemetry.addData("rightX2",rightX2);
        telemetry.addData("rightY2",rightY2);
        telemetry.addData("gpad1leftTrigger",gpad1leftTrigger);
        telemetry.addData("gpad1rightTrigger",gpad1rightTrigger);
        telemetry.addData("gpad2leftTrigger",gpad2leftTrigger);
        telemetry.addData("gpad2rightTrigger",gpad2rightTrigger);
        telemetry.update();
    }

    private void foundation() {
        if (gpad1x) {
            if (isUp) {
                foundationRight.setPosition(1);
                foundationLeft.setPosition(1);
                isUp = false;
            } else {
                foundationRight.setPosition(0.5);
                foundationLeft.setPosition(0.5);
                isUp=true;
            }
        }
    }
//
//    private void grabber() {
//
//        if (gpad2a) {
//            grabber.setPosition(0.7);
//
//        }
//        if (gpad2b) {
//            grabber.setPosition(0);
//        }
//    }

//    private void armForwardBack() {
//        if (gpad2x) {
//            armServo.setPower(0.7);
//
//        } else if (gpad2y) {
//            armServo.setPower(-0.7);
//        } else {
//            armServo.setPower(0);
//        }
//    }
//
//    private void armUpDown() {
//        if (gpad2rightTrigger > 0.5) {
//            armLeft.setPower(-1);
//            armRight.setPower(-1);
//
//        } else if (gpad2leftTrigger > 0.5) {
//            armLeft.setPower(1);
//            armRight.setPower(1);
//        } else {
//
//            armLeft.setPower(0);
//            armRight.setPower(0);
//        }
//    }


    // forward 1 -leftY
    // backward 1 -leftY
    // slide-right 1 - leftX
    // slide-left -1 - leftX
    // frontLeft < -0.5 --lx < -0.5 --ly
    // frontRight > 0.5 --lx < -0.5 --ly
    // bottomLeft < -0.5 --lx > 0.5 --ly
    // bottomRight >0.5 --lx > 0.5 --ly



    private void move() {
        double powerStrafe = 0.5;
        double powerStraight = 0.8;
        double powerRotate = 0.5;
        // frontLeft
        if (leftX1 < -0.5 && leftY1 < -0.5) {
            strafeLeftFront(powerStrafe);
        }
        // frontRight
        else if (leftX1 > 0.5 && leftY1 < -0.5) {
            strafeRightFront(powerStrafe);
        }
        //bottomLeft
        else if (leftX1 < -0.5 && leftY1 > 0.5) {
            strafeLeftBack(powerStrafe);
        }
        // bottomRight
        else if (leftX1 > 0.5 && leftY1 > 0.5) {
            strafeRightBack(powerStrafe);
        }
        // slideRight
        else if (leftX1 > 0.9) {
            slideRight(powerStrafe);
        }
        // slideLeft
        else if (leftX1 < -0.9) {
            slideLeft(powerStrafe);
        }
        // forward
        else if (leftY1 < -0.9) {
            forward(powerStraight);
        }
        // backward
        else if (leftY1 > 0.9) {
            backward(powerStraight);
        }
        // rotate right
        else if (rightX1 > 0.9) {
            rotateRight(powerRotate);
        }
        // rotate left
        else if (rightX1 < -0.9) {
            rotateLeft(powerRotate);
        }
        else {
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
        topRight.setPower(-power);
        bottomLeft.setPower(-power);
    }

    private void strafeLeftFront(double power) {
        topRight.setPower(power);
        bottomLeft.setPower(power);
    }

    private void strafeLeftBack(double power) {

        topLeft.setPower(-power);
        bottomRight.setPower(-power);
    }


}