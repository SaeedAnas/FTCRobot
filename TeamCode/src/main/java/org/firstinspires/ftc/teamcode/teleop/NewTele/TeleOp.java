package org.firstinspires.ftc.teamcode.teleop.NewTele;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOp extends LinearOpMode {

    protected static DcMotor topRight;

    protected static DcMotor topLeft;

    protected static DcMotor bottomLeft;

    protected static DcMotor bottomRight;

    protected static DcMotor armMotorRight;

    protected static DcMotor armMotorLeft;

    protected static DcMotor intakeLeft;

    protected static DcMotor intakeRight;

    protected static Servo foundationRight;

    protected static Servo foundationLeft;

    protected static Servo sideServo;

    protected static Servo grabber;

    protected static DcMotor cascadeRight;

    protected static DcMotor cascadeLeft;

    protected static CRServo blockMover;

    protected Buttons b;

    protected SparseArray<Button> buttons;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while(opModeIsActive()) {
            b.update();
            run();
        }
    }

    private void run() {
        for (int i = 0; i < buttons.size(); i++) {
            buttons.get(i).run();
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


        // armServo = hardwareMap.get(CRServo.class, "armServo");
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reverse right motors
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        cascadeRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        b = new Buttons();
        buttons = new SparseArray<>();
        buttons.append(0, new MechanumMovement());
        buttons.append(1, new Foundation());
        telemetry.addData("Status: ", "Ready");
        telemetry.update();
    }

}
