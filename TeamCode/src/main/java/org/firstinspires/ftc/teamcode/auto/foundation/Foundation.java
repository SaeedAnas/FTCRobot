package org.firstinspires.ftc.teamcode.auto.foundation;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Foundation extends Autonomous {
    // PLEASE READ:
    // ALWAYS CODE FOR BLUE TEAM AND ADD * turnVal to the turns
    // -Degree is left, +Degree is right

    private static int turnVal;

    private static void checkTeam(char team) {
        if (team == 'b')
            turnVal = 1;
        else if (team == 'r')
            turnVal = -1;
    }

    private void moveFoundationHorizontal(char team) {
        checkTeam(team);
        releaseFoundation();
        move(BACKWARD, 3, DRIVE_SPEED);
        turnByGyro(TURN_SPEED, -45 * turnVal);
        move(BACKWARD, 30, DRIVE_SPEED);
        turnByGyro(TURN_SPEED, 45 * turnVal);
        move(BACKWARD, TILE_LENGTH - 18, DRIVE_SPEED);
        grabFoundation();
        sleep(1000);
        move(FORWARD, 5, DRIVE_SPEED);
        turnByGyro(1, -90 * turnVal);
        releaseFoundation();
        move(FORWARD, TILE_LENGTH - 3, DRIVE_SPEED);
        brake();
    }

    private void moveFoundationRed() {
        releaseFoundation();
        autoCorrectMove(FORWARD_RIGHT, 20, 0.3);
        sleep(500);
        autoCorrectMove(FORWARD, 10, 0.3);
        sleep(500);
        grabFoundation();
        sleep(500);
        move(BACKWARD, 10, 0.3);
        sleep(500);
        turnByGyro(0.7, 90);
        sleep(500);
        releaseFoundation();
        sleep(500);
        move(BACKWARD, TILE_LENGTH, 0.3);
        sleep(500);
    }
    private void moveFoundationBlue() {
        releaseFoundation();
        autoCorrectMove(FORWARD_LEFT, 20, 0.3);
        sleep(500);
        autoCorrectMove(FORWARD, 10, 0.3);
        sleep(500);
        grabFoundation();
        sleep(500);
        move(BACKWARD, 10, 0.3);
        sleep(500);
        turnByGyro(0.7, -90);
        sleep(500);
        releaseFoundation();
        sleep(500);
        move(BACKWARD, TILE_LENGTH, 0.3);
        sleep(500);
    }
    // 10 foudnation
    // 20 skystone
    // 8
    // 10 both on the bridge


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

    void currentFoundation(char team) {
        if (team == 'r') {
            moveFoundationRed();
        } else if (team == 'b') {
            moveFoundationBlue();
        }
    }


}
