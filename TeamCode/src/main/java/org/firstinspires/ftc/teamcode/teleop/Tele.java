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
    private DcMotor left, right, armLeft, armRight;
    private Servo rightFoundation, leftFoundation, grabber;
    private CRServo armServo;

    // Mechanum
    private DcMotor bottomLeft, topLeft, bottomRight, topRight;

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
    private static boolean gpad1x, gpad1y, gpad2a, gpad2b, gpad2x, gpad2y, gpad2rightBumper, gpad2leftBumper, gpad1rightBumper, gpad1leftBumper;
    private static double leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, gpad1leftTrigger, gpad1rightTrigger, gpad2leftTrigger, gpad2rightTrigger;

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo = hardwareMap.get(CRServo.class, "armServo");

        // rightMotor is upside-down
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //zero power behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            getValues();
            foundation();
            grabber();
            armForwardBack();
            armUpDown();
            move();
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

    private void foundation() {
        if (gpad1x) {
            rightFoundation.setPosition(0.6);
            leftFoundation.setPosition(0.4);
        }
        if (gpad1y) {
            rightFoundation.setPosition(0.1);
            leftFoundation.setPosition(0.9);
        }
    }

    private void grabber() {

        if (gpad2a) {
            grabber.setPosition(0.7);

        }
        if (gpad2b) {
            grabber.setPosition(0);
        }
    }

    private void armForwardBack() {
        if (gpad2x) {
            armServo.setPower(0.7);

        } else if (gpad2y) {
            armServo.setPower(-0.7);
        } else {
            armServo.setPower(0);
        }
    }

    private void armUpDown() {
        if (gpad2rightTrigger > 0.5) {
            armLeft.setPower(-1);
            armRight.setPower(-1);

        } else if (gpad2leftTrigger > 0.5) {
            armLeft.setPower(1);
            armRight.setPower(1);
        } else {

            armLeft.setPower(0);
            armRight.setPower(0);
        }
    }

    private void move() {
        double switchVal = gpad1leftTrigger;

        if (leftY1 > 0) {
            forward(leftY1);
        } else if (leftY1 < 0) {
            backward(-leftY1);
        } else if (leftX1 > 0) {
            slideRight(leftX1);
        } else if (leftX1 < 0) {
            slideLeft(-leftX1);
        } else if (rightX1 > 0) {
            rotateRight(rightX1);
        } else if (rightX1 < 0) {
            rotateLeft(-rightX1);
        } else if (switchVal != 0) {
            if (rightY1 > 0) {
                strafeLeftFront(rightY1);
            } else if (rightY1 < 0) {
                strafeLeftBack(-rightY1);
            }
        } else if (rightY1 > 0) {
            strafeRightFront(rightY1);
        } else if (rightY1 < 0) {
            strafeRightBack(-rightY1);
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