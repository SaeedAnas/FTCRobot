package org.firstinspires.ftc.teamcode.auto.core;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {
    public static final double
            ROBOT_WIDTH = 17,
            ROBOT_LENGTH = 15.25,
            TILE_LENGTH = 24,
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 0.33333, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 3.93,
            WHEEL_WIDTH_INCHES = 1.77165,
            ROLLER_LENGTH_INCHES = 1.5748,
            //COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            COUNTS_PER_INCH = (1101+1132+1095+1084)/80,
            DRIVE_SPEED = 0.4,
            TURN_SPEED = 0.3,
            ARM_SPEED = 0.3,
            L_FOUNDATION_GRAB = 0,
            L_FOUNDATON_RELEASE = 0.4,
            R_FOUNDATION_GRAB = 0.26,
            R_FOUNDATION_RELEASE = 0.66,
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
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;

    public static final String VUFORIA_KEY =
            "AT8pGt3/////AAABmQ9LKBWthkikgQSErtn4C1GN+U/k35mErGuydnhrXtBLs2+wEnRYzMx2qJC0Q+4bHLUaWRZ18gRQcTZOoaKDYfG7yIcNfsexI4G5IdAgwfAZnSbrWco7IW2mdaHZrQ5mw/u0mh1RHbcPdK3JAheEknMP53n73JNNBFbEcB+IN2qPSI4AUrWqK3TuAl7XCnEBQrHKB7kU62rXWs+4r4/RcNB0g/yMZ3S5Yv7vfHYGMEA3/Wj+4PC/6v/pO9StgMjxKaVZMjTYiHvUN6yi6CgVfQlKlmkEMU0IR60PcUgA9hKq9CPXVNPN1tXCTGFGdd+WbhFEGdkbZxY3scU85G4kDQy2oNbFRaClpdHYINBOV1U1";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    // Class Members
    public static OpenGLMatrix lastLocation = null;
    public static VuforiaLocalizer vuforia = null;
    public static boolean targetVisible = false;
    public static float phoneXRotate    = 0;
    public static float phoneYRotate    = 0;
    public static float phoneZRotate    = 0;
}
