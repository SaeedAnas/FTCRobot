package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import org.firstinspires.ftc.teamcode.auto.vision.VisionPipeline;

import java.util.concurrent.Callable;
import java.util.concurrent.FutureTask;

import static org.firstinspires.ftc.teamcode.auto.core.Direction.*;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Block extends Autonomous {
    int degreesToMoveFoundation = 85;
    double moveToFoundation = (TILE_LENGTH * 2) - ROBOT_LENGTH - 2;
    double moveAfterFoundation = TILE_LENGTH + 6;
    double moveAfterRelease = 5;
    double moveToAvoidOtherRobot = 17;
    double moveToParkUnderBridge = ((TILE_LENGTH * 2) - (ROBOT_LENGTH+15)) * 1.2;
    int OneBlockLength = 8;
    Callable<Integer> blockMoverFront = new Callable<Integer>() {

        @Override
        public Integer call() throws Exception {
            moveServo(0.7);
            return 1;
        }
    };
    FutureTask<Integer> blockMoveFrontTask = new FutureTask<>(blockMoverFront);
    Thread blockMoverFrontThread = new Thread(blockMoveFrontTask);
    Callable<Integer> blockMoverBack = new Callable<Integer>() {

        @Override
        public Integer call() throws Exception {
            moveServo(0.7);
            return 1;
        }
    };
    FutureTask<Integer> blockMoveBackTask = new FutureTask<>(blockMoverBack);
    Thread blockMoverBackThread = new Thread(blockMoveBackTask);
    Callable<Integer> cascadeUp = new Callable<Integer>() {

        @Override
        public Integer call() throws Exception {
            moveCascades(1);
            return 1;
        }
    };
    FutureTask<Integer> cascadeUpTask = new FutureTask<>(cascadeUp);
    Thread cascadeUpThread = new Thread(cascadeUpTask);

    Callable<Integer> cascadeDown = new Callable<Integer>() {

        @Override
        public Integer call() throws Exception {
            moveCascades(-1);
            return 1;
        }
    };
    FutureTask<Integer> cascadeDownTask = new FutureTask<>(cascadeDown);
    Thread cascadeDownThread = new Thread(cascadeDownTask);
    int getter;

    public void getBlock(char team){
        try {

            scanMode();
            sleep(1000);
            int pos = VisionPipeline.getBlockPosition();
            off();
            telemetry.addData("pos:", pos);
            telemetry.update();

            sleep(1000);
            if (pos == 2) {
                first();
            } else if (pos == 1) {
                second();
            } else {
                third();
            }

        } catch (Exception e) {
            stopAllMotors();
        }
    }

    public void first() throws Exception{
        blockMoverFrontThread.start();
        releaseFoundation();
        dropBlock();
        correctMove(FORWARD, (TILE_LENGTH * 2) - ROBOT_WIDTH + 5, 0.5);
        getter = blockMoveFrontTask.get();
        pickUpBlock();
        sleep(1000);
        correctMove(BACKWARD, 6, 0.5);
        turnByGyro(0.5, 75);
        correctMove(FORWARD, TILE_LENGTH * 3, 0.5);
        cascadeUpThread.start();
        move(FORWARD,10, 0.8);
        turnByGyro(0.4,-70);
        move(FORWARD,7, 0.8);
        getter = cascadeUpTask.get();
        dropBlock();
        sleep(500);
        pickUpBlock();
        move(RIGHT, 15, 0.8);
        move(FORWARD, 4,0.8);
        sleep(1000);
        grabFoundation();
        sleep(1000);
        move(BACKWARD, moveAfterFoundation, DRIVE_SPEED);
        sleep(1000);
        turnByGyro(0.9, degreesToMoveFoundation);
        releaseFoundation();
        sleep(1000);
        move(FORWARD,5,DRIVE_SPEED);
        correctMove(LEFT, moveToAvoidOtherRobot, DRIVE_SPEED);
        cascadeDownThread.start();
        correctMove(BACKWARD, moveAfterRelease, DRIVE_SPEED);
        sleep(1000);
        getter = cascadeDownTask.get();
        correctMove(BACKWARD, moveToParkUnderBridge, DRIVE_SPEED);
        brake();

    }

    public void second() throws Exception{
        blockMoverFrontThread.start();
        dropBlock();
        correctMove(LEFT, OneBlockLength*1.7, 0.5);
        correctMove(FORWARD, (TILE_LENGTH * 2) - ROBOT_WIDTH + 5, 0.5);
        getter = blockMoveFrontTask.get();
        pickUpBlock();
        sleep(1000);
        correctMove(BACKWARD, 6, 0.5);
        turnByGyro(0.5, 75);
        correctMove(FORWARD, (TILE_LENGTH * 3) + OneBlockLength*1.7, 0.5);
        cascadeUpThread.start();
        turnByGyro(0.4,-70);
        move(FORWARD,13, 0.8);
        getter = cascadeUpTask.get();
        dropBlock();
        sleep(500);
        pickUpBlock();
        sleep(1000);
        grabFoundation();
        sleep(1000);
        correctMove(BACKWARD, moveAfterFoundation, DRIVE_SPEED);
        sleep(1000);
        turnByGyro(0.9, degreesToMoveFoundation);
        sleep(1000);
        releaseFoundation();
        sleep(1000);
        correctMove(BACKWARD, moveAfterRelease, DRIVE_SPEED);
        cascadeDownThread.start();
        correctMove(LEFT, moveToAvoidOtherRobot, DRIVE_SPEED);
        sleep(1000);
        getter = cascadeDownTask.get();
        correctMove(BACKWARD, moveToParkUnderBridge, DRIVE_SPEED);
        brake();

    }

    public void third() throws Exception{
        blockMoverFrontThread.start();
        dropBlock();
        correctMove(LEFT, OneBlockLength*3, 0.5);
        correctMove(FORWARD, (TILE_LENGTH * 2) - ROBOT_WIDTH + 5, 0.5);
        getter = blockMoveFrontTask.get();
        pickUpBlock();
        sleep(1000);
        correctMove(BACKWARD, 6, 0.5);
        turnByGyro(0.5, 75);
        correctMove(FORWARD, (TILE_LENGTH * 3) + (OneBlockLength*2), 0.5);
        cascadeUpThread.start();
        turnByGyro(0.4,-70);
        move(FORWARD,13, 0.8);
        getter = cascadeUpTask.get();
        dropBlock();
        sleep(500);
        pickUpBlock();
        sleep(1000);
        grabFoundation();
        sleep(1000);
        correctMove(BACKWARD, moveAfterFoundation, DRIVE_SPEED);
        sleep(1000);
        turnByGyro(0.9, degreesToMoveFoundation);
        sleep(1000);
        releaseFoundation();
        sleep(1000);
        correctMove(BACKWARD, moveAfterRelease, DRIVE_SPEED);
        cascadeDownThread.start();
        correctMove(LEFT, moveToAvoidOtherRobot, DRIVE_SPEED);
        sleep(1000);
        getter = cascadeDownTask.get();
        correctMove(BACKWARD, moveToParkUnderBridge, DRIVE_SPEED);
        brake();

    }


}
