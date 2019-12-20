package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {
    public static final double
            // ROBOT_WIDTH = 17,
            ROBOT_LENGTH = 15.25,
            TILE_LENGTH = 23.75,
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 0.33333, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 3.85827,
            WHEEL_WIDTH_INCHES = 1.77165,
            ROLLER_LENGTH_INCHES = 1.5748,
            COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            DRIVE_SPEED = 0.4,
            TURN_SPEED = 0.3,
            ARM_SPEED = 0.3,
            LEFT_FOUNDATION_DOWN = 0.5,
            LEFT_FOUNDATION_UP = 1,
            RIGHT_FOUNDATION_DOWN = 0.5,
            RIGHT_FOUNDATION_UP = 1,
            GRABBER_GRAB = 0.0,
            GRABBER_RELEASE = 0.7,
            GEAR_IN = 32,
            GEAR_OUT = 16,
            GEAR_RATIO_ARM = GEAR_OUT/GEAR_IN,
            P_DISTANCE_PER_ROTATION = 20.8,
            TICKS_PER_MM_ARM = (GEAR_RATIO_ARM * COUNTS_PER_MOTOR_REV)/P_DISTANCE_PER_ROTATION,
            CORRECTION = 3,
            DEGREE_THRESHOLD = 3.5,
            TICKS_PER_INCH_STRAIGHT = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    protected static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    protected static final String VUFORIA_KEY =
            "AVkAyx3/////AAABmYdyw7V6PUSel7U9ewtEEL2IhIh/Tm3oN+WUruzFFBvM0UxDGV568bm9ypYMLndtLha14b/MV+wy655J6ERCXJhCtur/UbPnP8oB84ZVjBj1FBK1V9lPumOu8zxDJejYGh2qVyOjhpuMvSmhBncps70LW8cTEMPeMZNS4q81D5AzsYqmo0JeJ4CEiMnBgrsbbqUxsJIqhDYf2GPbwuYSSONMwJF8dfBDUyjA29uE+VeLEikATT842Kev7CGtYmSEmej0HhZDxkoiTM5D8tHfkpxQ1ApLp5oeWv96ihTw7z3/ILnQv/8IlNvVb31EVl3f0TVBBKxUy6dITvsTpLQMp684qMgx9EJRBP+imJjC9HtN";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    protected static final float mmPerInch        = 25.4f;
    protected static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    protected static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    protected static final float bridgeZ = 6.42f * mmPerInch;
    protected static final float bridgeY = 23 * mmPerInch;
    protected static final float bridgeX = 5.18f * mmPerInch;
    protected static final float bridgeRotY = 59;                                 // Units are degrees
    protected static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    protected static final float halfField = 72 * mmPerInch;
    protected static final float quadField  = 36 * mmPerInch;

    // Class Members
    public static OpenGLMatrix lastLocation = null;
    public static VuforiaLocalizer vuforia = null;
    public static boolean targetVisible = false;
    public static float phoneXRotate    = 0;
    public static float phoneYRotate    = 0;
    public static float phoneZRotate    = 0;
}
