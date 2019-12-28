package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import org.firstinspires.ftc.teamcode.auto.vision.VisionPipeline;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.LEFT;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.RIGHT;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Block extends Autonomous {
    // PLEASE READ:
    // ALWAYS CODE FOR BLUE TEAM AND ADD * turnVal to the turns
    // -Degree is left, +Degree is right

    // blue
    public void getBlock(char team){
        int pos = VisionPipeline.getBlockPosition();
        one(team);
        // if (pos == 0) {
        //     one(team);
        // } else if (pos == 1) {
        //     two(team);
        // } else if (pos == 2) {
        //     three(team);
        // }
    }
    // Go to the first block
    // Get the first block
    // Go to foundation
    // Put the first block
    // Go to the second block
    // Get the second block
    // Go to foundation
    // Put the second block
    // Grab Foundaton
    // Go Back
    // Turn to the wall
    // Push foundation into the build site
    int OneBlockLength = 8;
    double d1Block = (TILE_LENGTH*2)-ROBOT_WIDTH - 3;
    double d2Block = (TILE_LENGTH*2)-ROBOT_WIDTH;
    double d3Block = (TILE_LENGTH * 2) - ROBOT_WIDTH;
    // 69
    double d1Foundation = 47.5;
    double d2Foundation = d1Foundation + (OneBlockLength);
    double d3Foundation = d1Foundation + (OneBlockLength*2);
    double d4Foundation = d1Foundation + (OneBlockLength*3);
    double d5Foundation = d1Foundation + (OneBlockLength*4);
    double d6Foundation = d1Foundation + (OneBlockLength*5);
    int space = 6;

    private void moveToFoundation(double distance, char team) {
        if(team == 'b') {
            move(FORWARD, distance, DRIVE_SPEED);
            move(RIGHT, 2, DRIVE_SPEED);
            sleep(2000);
            dropBlock();
            move(LEFT, space, DRIVE_SPEED);
        } else if (team == 'r') {
            move(FORWARD, distance,DRIVE_SPEED);
            move(LEFT, space, DRIVE_SPEED);
            dropBlock();
            move(RIGHT, space, DRIVE_SPEED);
        }
    }

    private void moveToBlock(double distance, char team) {
        if (team == 'b') {
            move(BACKWARD,distance, DRIVE_SPEED);
            move(RIGHT,space + 0.3, DRIVE_SPEED);
            sleep(2000);
            pickUpBlock();
            move(LEFT,space,DRIVE_SPEED);
        } else if (team == 'r') {
            move(BACKWARD, distance, DRIVE_SPEED);
            move(LEFT, space, DRIVE_SPEED);
            pickUpBlock();
            move(RIGHT, space,DRIVE_SPEED);
        }
    }

    private void moveFoundation(char team) {
        if (team == 'b') {
            turnByGyro(TURN_SPEED, 90);
            move(BACKWARD,(TILE_LENGTH*2)-ROBOT_LENGTH, 1);
            turnByGyro(TURN_SPEED, -90);
            move(FORWARD, 5, 1);
            move(BACKWARD_LEFT, 15, DRIVE_SPEED);
            move(BACKWARD, 15, DRIVE_SPEED);
        } else if (team == 'r') {
            turnByGyro(TURN_SPEED, -90);
            move(BACKWARD, (TILE_LENGTH*2)-ROBOT_LENGTH, 1);
            move(FORWARD, TILE_LENGTH, 1);
            turnByGyro(TURN_SPEED, 90);
            move(BACKWARD, 15, DRIVE_SPEED);
            move(FORWARD, 15, DRIVE_SPEED);
        }
    }

    private void toFirst(char team) {
        int distance = 10;
        if (team =='b') {
        move(RIGHT, d1Block, DRIVE_SPEED);
        move(FORWARD, OneBlockLength, 0.3);
        sleep(2000);
        pickUpBlock();
        move(LEFT, space, DRIVE_SPEED);
        } else if (team == 'r') {
        move(FORWARD_LEFT, d1Block, DRIVE_SPEED);
        move(LEFT, distance, DRIVE_SPEED);
        pickUpBlock();
        move(RIGHT, space, DRIVE_SPEED);
        }
    }

    private void toSecond(char team) {
        int distance = 15;
        if (team == 'b') {
        move(FORWARD_RIGHT, d2Block, DRIVE_SPEED);
        move(RIGHT, 4, DRIVE_SPEED);
        pickUpBlock();
        move(FORWARD, space, DRIVE_SPEED);
        } else if (team == 'r') {
            move(FORWARD_LEFT, d2Block, DRIVE_SPEED);
            move(LEFT, distance, DRIVE_SPEED);
            pickUpBlock();
            move(RIGHT, space, DRIVE_SPEED);
        }
    }

    private void toThird(char team) {
        if (team == 'b') {
        move(RIGHT, d3Block,DRIVE_SPEED);
        pickUpBlock();
        move(LEFT,space,DRIVE_SPEED);
        } else if (team == 'r') {
            move(LEFT,d3Block, DRIVE_SPEED);
            pickUpBlock();
            move(RIGHT, space, DRIVE_SPEED);
        }
    }



    private void firstToFoundation(char team) { moveToFoundation(d1Foundation,team); }

    private void secondToFoundation(char team) { moveToFoundation(d2Foundation, team); }

    private void thirdToFoundation(char team) { moveToFoundation(d3Foundation, team); }





    private void foundationToFourth(char team) {
            moveToBlock(d4Foundation, team);
    }

    private void foundationToFifth(char team) { moveToBlock(d5Foundation,team); }

    private void foundationToSixth(char team) {
        moveToBlock(d6Foundation, team);
    }

    private void fourthToFoundation(char team) {
        moveToFoundation(d4Foundation, team);
    }

    private void fifthToFoundation(char team) {
        moveToFoundation(d5Foundation, team);
    }

    private void sixthToFoundation(char team) {
        moveToFoundation(d6Foundation, team);
    }

    private void one(char team) {
        toFirst(team);
        firstToFoundation(team);
        foundationToFourth(team);
        fourthToFoundation(team);
        moveFoundation(team);
    }

    private void two(char team) {
        toSecond(team);
        secondToFoundation(team);
        foundationToFifth(team);
        fifthToFoundation(team);
        moveFoundation(team);
    }

    private void three(char team) {
        toThird(team);
        thirdToFoundation(team);
        foundationToSixth(team);
        sixthToFoundation(team);
        moveFoundation(team);
    }







































































































































































































































































































































































































    private void dickbutt() {
        telemetry.addData("Status: ", "Dicking Butt");
        telemetry.update();
    }
}







































































































































































































































































































































































































