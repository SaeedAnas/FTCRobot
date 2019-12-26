package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
import org.firstinspires.ftc.teamcode.auto.vision.VisionPipeline;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

abstract class Block extends Autonomous {
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

    private static void getBlock(char team){
        VisionPipeline.getBlockPosition(team);
    }


     void currentBlock(char team) {

    }
}
