package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import org.firstinspires.ftc.teamcode.auto.vision.VisionPipeline;

import static org.firstinspires.ftc.teamcode.auto.core.Direction.*;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Block extends Autonomous {
    // PLEASE READ:
    // ALWAYS CODE FOR BLUE TEAM AND ADD * turnVal to the turns
    // -Degree is left, +Degree is right

    // blue
    public void getBlock(char team){
        scanMode();
        //int pos = VisionPipeline.getBlockPosition();
        int pos = 1;
        off();
        moveServo(0.7);
        dropBlock();
        move(FORWARD, (TILE_LENGTH*2)-ROBOT_WIDTH + 5, 0.5);
        sleep(1000);
        pickUpBlock();
        sleep(1000);
        move(BACKWARD, 10, 0.5);
        move(RIGHT, 25,0.5);

////        one(team);
//         if (pos == 0) {
//             one(team);
//         } else if (pos == 1) {
//             two(team);
//         } else if (pos == 2) {
//             three(team);
//         }
    }

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

    private void moveToFoundation(double distance, char team){
        if(team == 'b') {
            sideServo.setPosition(0.1);
            move(FORWARD, distance, DRIVE_SPEED);
            move(RIGHT, 2, DRIVE_SPEED);
            sleep(1000);
            dropBlock();
            move(LEFT, space, DRIVE_SPEED);
        } else if (team == 'r') {
            sideServo.setPosition(0.1);
            move(FORWARD, distance,DRIVE_SPEED);
            move(LEFT, space, DRIVE_SPEED);
            dropBlock();
            move(RIGHT, space, DRIVE_SPEED);
        }
    }

    private void moveToBlock(double distance, char team) {
        if (team == 'b') {
            sideServo.setPosition(0.1);
            move(BACKWARD,distance, 0.5);
            move(RIGHT, space + 0.3, DRIVE_SPEED);
            sleep(1000);
            pickUpBlock();
            move(LEFT,space,DRIVE_SPEED);
        } else if (team == 'r') {
            sideServo.setPosition(0.1);
            move(BACKWARD, distance, 0.5);
            move(FORWARD_LEFT, space, 0.5);
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
        move(FORWARD, OneBlockLength, 0.5);
        sleep(1000);
        pickUpBlock();
        move(LEFT, space, DRIVE_SPEED);
        } else if (team == 'r') {
        move(FORWARD, d1Block, 0.5);
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





    private void foundationToFourth(char team) { moveToBlock(d4Foundation, team); }

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
}
