package org.firstinspires.ftc.teamcode.auto.foundation;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.LEFT;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.RIGHT;
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


    private void moveFoundationRed() {
        try {
            releaseFoundation();
            releaseFoundation();
            move(FORWARD, (TILE_LENGTH * 2) - ROBOT_LENGTH - 2, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, TILE_LENGTH + 3, 1);
            turnByGyro(0.9, 85);
            releaseFoundation();
            move(BACKWARD, 5, DRIVE_SPEED);
            move(RIGHT, 20, DRIVE_SPEED);
            sleep(1000);
            move(BACKWARD, ((TILE_LENGTH * 2) - (ROBOT_LENGTH+10)) * 1.2, DRIVE_SPEED);
            brake();
    } catch (Exception e) {
        telemetry.addData("F", "IT DED");
        telemetry.update();
    }
    }
    private void moveFoundationBlue() {
        try {
            releaseFoundation();
            releaseFoundation();
            move(FORWARD, (TILE_LENGTH * 2+3) - ROBOT_LENGTH - 2, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, TILE_LENGTH + 6, 1);
            turnByGyro(0.9, -85);
            releaseFoundation();
            sleep(1000);
            move(BACKWARD, 5, DRIVE_SPEED);
            move(RIGHT, 17, DRIVE_SPEED);
            sleep(1000);
            autoCorrectMove(BACKWARD, ((TILE_LENGTH * 2) - (ROBOT_LENGTH+5)) * 1.2, DRIVE_SPEED);
            brake();
        } catch (Exception e) {
            telemetry.addData("F", "IT DED");
            telemetry.update();
        }
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
