package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static org.opencv.imgproc.Imgproc.ADAPTIVE_THRESH_MEAN_C;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;

public class VisionPipeline extends OpenCvPipeline{
    // THIS DETECTOR RETURNS THE PIXEL LOCATION OF THE LEFT MOST BOUNDARY OF THE BLACK TARGET
    // YOU CAN EASILY MODIFY IT TO GET YOU THE CENTER
    // IT IS YOUR JOB TO DETERMINE WHERE YOU WANT TO GO BASED ON THIS VALUE

    // SOMETIMES THERE IS SOME RANDOM CRASH THAT HAPPENS IF UR CAMERA WOBBLES A LOT
    // I AM LOOKING INTO A FIX, BUT AS LONG AS UR CAMERA DOESN"T FLAIL WILDLY IN MATCH U R GOOD

    // IF YOU SEE BUGS PLEASE MESSAGE ME ON DISCORD at Epsilon#0036
    // THIS IS NO MEANS ROBUST, BUT IT WORKS WELL FOR ME

    // THIS DETECTOR SACRIFICES SPEED FOR A LOT OF VERSATILITY
    // IT FUNCTIONS WITH LOTS OF NOISE BY PERFORMING LOTS OF FILTERS
    // IF YOU FEEL THAT SOME PARTS OF THIS PIPELINE ARENT NEEDED, THEN REMOVE THEM
    // TO IMPROVE FRAMERATE


    private final Scalar minHSV = new Scalar(11.8, 161.7, 116.5);
    private final Scalar maxHSV = new Scalar(30.3, 255.0, 255.0);

    private final Point anchor = new Point(-1,-1);
    private final int erodeIterations = 10;

    private final int dilateIterations = 20;

    // THESE NEED TO BE TUNED BASED ON YOUR DISTANCE FROM THE BLOCKS
    private final double minContourArea = 300.0;
    private final double minContourPerimeter = 1000.0;
    private final double minContourWidth = 300.0;
    private final double minContourHeight = 0.0;

    private final double cbMin = 105;
    private final double cbMax = 140;

    private int minX, minY = Integer.MAX_VALUE;
    private int maxX, maxY = -1 * Integer.MAX_VALUE;

    // TUNE THESE THEY WILL VARY BASED ON WEBCAM PLACEMENT!!!!!

    private final int maxVumarkValue = 80; // used to be 150
    private final int valleyLength = 40;

    private int vumarkLeftBoundary = -1;

    private Mat mask = new Mat();
    private Mat kernel = new Mat();


    private Mat hierarchy = new Mat();

    // private CSVWriter csvWriter = new CSVWriter(new File("colsums.java"));


    private final int INDEX_ERROR = -2; // index error code

    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2GRAY);
            //Imgproc.adaptiveThreshold(input, input, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 101, 40);
            Imgproc.threshold(input,input,25,255, THRESH_BINARY_INV);
            return input;
        } catch (Exception e) {
            return input;
        }
    }


    private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
        Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
        return new Mat(image, cropRect);
    }

    private Mat crop(Mat image, Rect rect) {
        return new Mat(image, rect);
    }


    private Rect getMaxRectangle() { return new Rect(new Point(minX, minY), new Point(maxX,maxY)); }

}
