package org.firstinspires.ftc.teamcode.teleop.NewTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

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

    protected static CRServo blockMover;

    protected static boolean gpad1x, gpad1y, gpad1a, gpad1b, gpad2a, gpad2b, gpad2x, gpad2y, gpad2rightBumper, gpad2leftBumper, gpad1rightBumper, gpad1leftBumper, dpad2Up, dpad2Down, dpad2Right, dpad2Left, dpad1Up, dpad1Down, dpad1Right, dpad1Left;
    protected static double leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, gpad1leftTrigger, gpad1rightTrigger, gpad2leftTrigger, gpad2rightTrigger;
    public OpMode opmode = this;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while(opModeIsActive()) {
            tele();
            update();
            accurateMove();
            foundation();
            intake();
            cascade();
            autoArm();
            grabber();
            continuousRack();
        }
    }



    void initHardware() {
        telemetry.addData("Status: ", "Initializing");
        telemetry.update();
        topRight = hardwareMap.get(DcMotor.class, "frontRight");
        topLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "rearRight");
        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        sideServo = hardwareMap.get(Servo.class, "sideServo");
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
//        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightMotor is upside-down
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        armMotorRight.setDirection(DcMotor.Direction.REVERSE);
        // reset the encoder
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
        telemetry.addData("Status: ", "Ready");
        telemetry.update();
        waitForStart();
    }
    protected  void tele(){
        telemetry.addData("frontleft",topLeft.getCurrentPosition());
        telemetry.addData("frontright",topRight.getCurrentPosition());
        telemetry.addData("bottomleft",bottomLeft.getCurrentPosition());
        telemetry.addData("bottomright",bottomRight.getCurrentPosition());
        telemetry.update();

    }

    protected void update() {
        gpad1x = opmode.gamepad1.x;
        gpad1y = opmode.gamepad1.y;
        gpad1a = opmode.gamepad1.a;
        gpad2b = opmode.gamepad2.b;
        gpad2a = opmode.gamepad2.a;
        gpad2b = opmode.gamepad1.b;
        gpad2x = opmode.gamepad2.x;
        gpad2y = opmode.gamepad2.y;
        gpad1leftTrigger = opmode.gamepad1.left_trigger;
        gpad1rightTrigger =opmode.gamepad1.right_trigger;
        gpad2leftTrigger = opmode.gamepad2.left_trigger;
        gpad2rightTrigger = opmode.gamepad2.right_trigger;
        gpad1leftBumper = opmode.gamepad1.left_bumper;
        gpad1rightBumper = opmode.gamepad1.right_bumper;
        gpad2rightBumper = opmode.gamepad2.right_bumper;
        gpad2leftBumper = opmode.gamepad2.left_bumper;
        leftX1 = opmode.gamepad1.left_stick_x;
        leftY1 = opmode.gamepad1.left_stick_y;
        rightX1 = opmode.gamepad1.right_stick_x;
        rightY1 = opmode.gamepad1.right_stick_y;
        leftX2 = opmode.gamepad2.left_stick_x;
        leftY2 = opmode.gamepad2.left_stick_y;
        rightX2 =opmode. gamepad2.right_stick_x;
        rightY2 =opmode.gamepad2.right_stick_y;
        dpad2Up = opmode.gamepad2.dpad_up;
        dpad2Down = opmode.gamepad2.dpad_down;
        dpad2Right = opmode.gamepad2.dpad_right;
        dpad2Left = opmode.gamepad2.dpad_left;
        dpad1Up = opmode.gamepad1.dpad_up;
        dpad1Down = opmode.gamepad1.dpad_down;
        dpad1Left = opmode.gamepad1.dpad_left;
        dpad1Right = opmode.gamepad1.dpad_right;
    }

    private void continuousRack() {
        if (leftY2 > 0) {
            blockMover.setPower(0.7);
        } else if (leftY2 < 0) {
            blockMover.setPower(-0.7);
        } else {
            blockMover.setPower(0);
        }
    }

    private void grabber() {
        if (gpad2x) {
            grabber.setPosition(0.4);
        } else if (gpad2y) {
            grabber.setPosition(0.9);
        }
    }

    private void foundation() {
        if (gpad1x) {
            foundationRight.setPosition(0.15);
            foundationLeft.setPosition(0.0);
        } else if (gpad1y) {
            foundationRight.setPosition(0.65);
            foundationLeft.setPosition(0.4);
        }
    }

    private void intake() {
        if (gpad1rightBumper) {
            intakeLeft.setPower(0.8);
            intakeRight.setPower(0.8);
        } else if (gpad1leftBumper){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    private void cascade() {
        if (dpad2Up) {
            cascadeLeft.setPower(0.5);
            cascadeRight.setPower(0.5);
        } else if (dpad2Down) {
            cascadeLeft.setPower(-0.5);
            cascadeRight.setPower(-0.5);
        } else {
            cascadeLeft.setPower(0);
            cascadeRight.setPower(0);
        }
    }

    private void autoArm() {
        if (dpad1Up) {
            sideServo.setPosition(1);
        } else if (dpad1Down) {
            sideServo.setPosition(0);
        }
    }

    private void stopRobot() {
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }


    //peter move function code variables
    public void accurateMove() {
        double y = -gamepad1.left_stick_y; // reversed
        double x = gamepad1.left_stick_x*1;//STRAFE FIX
        double rx = gamepad1.right_stick_x;

        double frontLeftPower=(y + x + rx);
        double frontRightPower=(y - x - rx);
        double backLeftPower=(y - x + rx);
        double backRightPower=(y + x - rx);
        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 || Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
            bottomRight.setPower(backRightPower);
            bottomLeft.setPower(backLeftPower);
            topRight.setPower(frontRightPower);
            topLeft.setPower(frontLeftPower);

        }

        else{
            bottomRight.setPower(backRightPower);
            bottomLeft.setPower(backLeftPower);
            topRight.setPower(frontRightPower);
            topLeft.setPower(frontLeftPower);
        }
    }
}


//     private void forward(double power) {
//         topRight.setPower(power);
//         topLeft.setPower(power);
//         bottomRight.setPower(power);
//         bottomLeft.setPower(power);
//     }

//     private void backward(double power) {
//         topRight.setPower(-power);
//         topLeft.setPower(-power);
//         bottomRight.setPower(-power);
//         bottomLeft.setPower(-power);
//     }

//     private void slideLeft(double power) {
//         topRight.setPower(power);
//         topLeft.setPower(-power);
//         bottomRight.setPower(-power);
//         bottomLeft.setPower(power);
//     }

//     private void slideRight(double power) {
//         topRight.setPower(-power);
//         topLeft.setPower(power);
//         bottomRight.setPower(power);
//         bottomLeft.setPower(-power);
//     }

//     private void rotateRight(double power) {
//         topRight.setPower(-power);
//         topLeft.setPower(power);
//         bottomRight.setPower(-power);
//         bottomLeft.setPower(power);
//     }

//     private void rotateLeft(double power) {
//         topRight.setPower(power);
//         topLeft.setPower(-power);
//         bottomRight.setPower(power);
//         bottomLeft.setPower(-power);
//     }

//     private void strafeRightFront(double power) {
//         topLeft.setPower(power);
//         bottomRight.setPower(power);
//     }

//     private void strafeRightBack(double power) {
//         topRight.setPower(-power);
//         bottomLeft.setPower(-power);
//     }

//     private void strafeLeftFront(double power) {
//         topRight.setPower(power);
//         bottomLeft.setPower(power);
//     }

//     private void strafeLeftBack(double power) {

//         topLeft.setPower(-power);
//         bottomRight.setPower(-power);
//     }

// }

// private void move() {
//     double powerStrafe = 0.5;
//     double powerStraight = 0.8;
//     double powerRotate = 0.5;
//     // frontLeft
//     if (leftX1 < -0.3 && leftY1 < -0.3) {
//         strafeLeftFront(powerStrafe);
//     }
//     // frontRight
//     else if (leftX1 > 0.3 && leftY1 < -0.3) {
//         strafeRightFront(powerStrafe);
//     }
//     //bottomLeft
//     else if (leftX1 < -0.3 && leftY1 > 0.3) {
//         strafeLeftBack(powerStrafe);
//     }
//     // bottomRight
//     else if (leftX1 > 0.3 && leftY1 > 0.3) {
//         strafeRightBack(powerStrafe);
//     }
//     // rotateRight
//     else if (leftX1 > 0.9) {
//         rotateRight(powerRotate);
//     }
//     // rotateLeft
//     else if (leftX1 < -0.9) {
//         rotateLeft(powerRotate);
//     }
//     // forward
//     else if (leftY1 < -0.9) {
//         forward(powerStraight);
//     }
//     // backward
//     else if (leftY1 > 0.9) {
//         backward(powerStraight);
//     }
//     // side right
//     else if (rightX1 > 0.9) {
//         slideRight(powerRotate);
//     }
//     // slide left
//     else if (rightX1 < -0.9) {
//         slideLeft(powerRotate);
//     } else {
//         stopRobot();
//     }

// }