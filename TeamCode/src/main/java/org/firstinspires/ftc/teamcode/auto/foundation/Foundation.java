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

<<<<<<< HEAD
    private void moveFoundationHorizontal(char team) {
        checkTeam(team);
        releaseFoundation();
        move(FORWARD, (TILE_LENGTH*2)-ROBOT_LENGTH, DRIVE_SPEED);
        grabFoundation();
        move(BACKWARD, TILE_LENGTH, DRIVE_SPEED);
        turnByGyro(TURN_SPEED, 115);
        move(FORWARD, 5, DRIVE_SPEED);
        releaseFoundation();
        sleep(1000);
        move(BACKWARD, (TILE_LENGTH*2)-ROBOT_LENGTH, DRIVE_SPEED);
        brake();
    }
=======
>>>>>>> 842c7d9bfb02ca81baf8c78463ea410686d16e6a

    private void moveFoundationRed() {
        try {
            releaseFoundation();
            releaseFoundation();
            move(FORWARD, (TILE_LENGTH * 2) - ROBOT_LENGTH - 2, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, TILE_LENGTH + 3, 1);
            turnByGyro(0.9, 90);
            releaseFoundation();
            move(FORWARD, 10, 1);
            sleep(1000);
            move(BACKWARD, ((TILE_LENGTH * 2) - ROBOT_LENGTH) * 1.2, DRIVE_SPEED);
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
            move(FORWARD, (TILE_LENGTH * 2) - ROBOT_LENGTH - 2, DRIVE_SPEED);
            grabFoundation();
            sleep(1000);
            move(BACKWARD, TILE_LENGTH + 3, 1);
            turnByGyro(0.9, -90);
            releaseFoundation();
            move(FORWARD, 10, 1);
            sleep(1000);
            move(BACKWARD, ((TILE_LENGTH * 2) - ROBOT_LENGTH) * 1.2, DRIVE_SPEED);
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
