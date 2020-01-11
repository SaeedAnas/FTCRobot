package org.firstinspires.ftc.teamcode.auto.foundation;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import java.sql.Driver;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.LEFT;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.RIGHT;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Foundation extends Autonomous {
    // PLEASE READ:
    // ALWAYS CODE FOR BLUE TEAM AND ADD * turnVal to the turns
    // -Degree is left, +Degree is right

    int degreesToMoveFoundation = 85;
    double moveToFoundation = (TILE_LENGTH * 2) - ROBOT_LENGTH - 2;
    double moveAfterFoundation = TILE_LENGTH + 6;
    double moveAfterRelease = 5;
    double moveToAvoidOtherRobot = 17;
    double moveToParkUnderBridge = ((TILE_LENGTH * 2) - (ROBOT_LENGTH+15)) * 1.2;

    private void moveFoundationRed() {
        try {
            releaseFoundation();
            releaseFoundation();
            move(FORWARD, moveToFoundation, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, moveAfterFoundation, DRIVE_SPEED);
            turnByGyro(0.9, degreesToMoveFoundation);
            releaseFoundation();
            sleep(1000);
            move(BACKWARD, moveAfterRelease, DRIVE_SPEED);
            move(LEFT, moveToAvoidOtherRobot, DRIVE_SPEED);
            sleep(1000);
            move(BACKWARD, moveToParkUnderBridge, DRIVE_SPEED);
            brake();
    } catch (Exception e) {
        telemetry.addData("F", "You fucking Donkey");
        telemetry.update();
    }
    }
    private void moveFoundationBlue() {
        try {
            releaseFoundation();
            releaseFoundation();
            move(FORWARD, moveToFoundation, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, moveAfterFoundation, DRIVE_SPEED);
            turnByGyro(0.9, -degreesToMoveFoundation);
            releaseFoundation();
            sleep(1000);
            move(BACKWARD, moveAfterRelease, DRIVE_SPEED);
            move(RIGHT, moveToAvoidOtherRobot, DRIVE_SPEED);
            sleep(1000);
            autoCorrectMove(BACKWARD, moveToParkUnderBridge, DRIVE_SPEED);
            brake();
        } catch (Exception e) {
            telemetry.addData("F", "You fucking Donkey");
            telemetry.update();
        }
    }

    void currentFoundation(char team) {
        if (team == 'r') {
            moveFoundationRed();
        } else if (team == 'b') {
            moveFoundationBlue();
        }
    }


//    private static int turnVal;
//
//    private static void checkTeam(char team) {
//        if (team == 'b')
//            turnVal = 1;
//        else if (team == 'r')
//            turnVal = -1;
//    }
//    private void moveFoundationVertical(char team) {
//        releaseFoundation();
//        drive(-DRIVE_SPEED, -(TILE_LENGTH + 12));
//        grabFoundation();
//        sleep(1000);
//        drive(DRIVE_SPEED, TILE_LENGTH + 7);
//        releaseFoundation();
//        turnByGyro(TURN_SPEED, -100 * turnVal);
//        drive(DRIVE_SPEED, ROBOT_LENGTH);
//        turnByGyro(TURN_SPEED, -100 * turnVal);
//        drive(DRIVE_SPEED, ROBOT_LENGTH + 15);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH * 2);
//        brake();
//    }
}
