package org.firstinspires.ftc.teamcode.auto.vision;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.TextView;

import com.vuforia.Image;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import static org.firstinspires.ftc.robotcontroller.internal.SingletonManager.singleton;
import static org.opencv.core.CvType.*;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;

public class VisionPipeline {

    private static final double NULL = -1.0; // Reset Integral Value
    private static final int LEFT = 0; // index for LEFT position value in Integrals
    private static final int MIDDLE = 1; // index for MIDDLE position value in Integrals
    private static final int RIGHT = 2; // index for RIGHT position value in Integrals

    // Corresponds to the area of interest for integral search
    private static final int BLOCK_HEIGHT = 15;

    // dimensions for the final image
    private static final int imgHeight = 500;
    private static final int imgWidth = 250;

    // TextView to display the integral values
    private TextView mat = singleton.getMatView();

    // array of integral values : index 0 = LEFT, index 1 = MIDDLE, index 2 = RIGHT
    private double[] integrals = new double[3];

    // last Integral array before reset
    // Used by other classes to get the largest value;
    private static double[] lastIntegrals = new double[3];

    // String printed in the TextView
    private static String lastIntegralString;

    VisionPipeline() {
        resetIntegrals();
    }

    /**
     * Gets the highest value in lastIntegrals and returns which position is most likely to have the block.
     * Should only be used when the first three blocks are in the camera's view.
     * @return either left, middle, or right depending on which one has the skystone
     */
    public static int getBlockPosition() {
        return getLargest(lastIntegrals);
    }

    /**
     * Takes an image, processes the image, and scales the image for the stream view
     * @param img image from the vuforia camera stream
     * @return Bitmap for the ImageView
     */
    Bitmap processImage(Image img) {

        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());

        Mat mat = bitmapToMat(bm, CvType.CV_8UC3);

        Mat ret = processFrame(mat);

        Bitmap retBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);

        Utils.matToBitmap(ret, retBitmap);

        return scaleImage(retBitmap);

    }

    /**
     * Processes the image into grayscale and extracts the integral values from the regions of intrest
     * @param input image from the camera in an opencv Mat object for easier processing
     * @return processed black white image
     */
    private Mat processFrame(Mat input) {

        try {
            /* Vuforia camera streams images that are rotated 90 degrees counterclockwise,
               so we have to rotate it 90 degrees clockwise so it is upright
            */
            Core.rotate(input, input, Core.ROTATE_90_CLOCKWISE);


            // turns image into grayscale
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

            // turns image into black and white
            Imgproc.threshold(input, input, 25, 255, THRESH_BINARY_INV);

            // gets the integral values
            getIntegralSums(input);

            return input;

        } catch (Exception e) {
            return input;
        }

    }

    /**
     * Extracts the integral sums of the 3 regions of intrest and stores them in the array Integrals
     * @param input black and white image
     */
    private void getIntegralSums(Mat input) {

        Mat left, middle, right;

        // divides the picture into 3 parts
        left = input.clone().submat(new Rect(new Point(0,0), new Point(input.cols()/3, input.rows())));
        middle = input.clone().submat(new Rect(new Point(input.cols()/3, 0), new Point((input.cols()/3)*2, input.rows())));
        right = input.clone().submat(new Rect(new Point((input.cols()/3)*2, 0), new Point(input.cols(), input.rows())));


        // narrows focus to the blocks
        left = left.submat(new Rect(new Point(0,((left.rows()/8)*5)-(left.rows()/BLOCK_HEIGHT)), new Point(left.cols(), ((left.rows()/8)*5)+(left.rows()/BLOCK_HEIGHT))));
        middle = middle.submat(new Rect(new Point(0,((middle.rows()/8)*5)-(middle.rows()/BLOCK_HEIGHT)), new Point(middle.cols(), ((middle.rows()/8)*5)+(middle.rows()/BLOCK_HEIGHT))));
        right = right.submat(new Rect(new Point(0,((right.rows()/8)*5)-(right.rows()/BLOCK_HEIGHT)), new Point(right.cols(), ((right.rows()/8)*5)+(right.rows()/BLOCK_HEIGHT))));

        // Multithread the integral calculations to make it go faster
        Thread l = new Thread(new MatSearcher(left, LEFT));
        Thread m = new Thread(new MatSearcher(middle, MIDDLE));
        Thread r = new Thread(new MatSearcher(right, RIGHT));
        l.start();
        m.start();
        r.start();
        waitForThread();

        // Creates and prints the integral values
        createString();
        updateTextView(lastIntegralString);

        resetIntegrals();

        // Draw the lines to show where the integral is taking place
        splitMat(input);
    }

    /**
     * Thread that searches a Matrix for the integral sum
     */
    class MatSearcher implements Runnable {

        private Mat division;
        private int index;

        /**
         * Constructor for the Thread
         * @param division Matrix that we want to find the integral of
         * @param index position of the matrix (LEFT, MIDDLE, RIGHT)
         */
        MatSearcher(Mat division, int index) {
            this.division = division;
            this.index = index;
        }

        @Override
        public void run() {
            getIntegralSum(division, index);
        }

    }

    /**
     * Finds the integralSum and stores it in the correct index of the integrals array
     * @param input Matrix that we want to find the integral of
     * @param index position of the matrix (LEFT, MIDDLE, RIGHT)
     */
    private void getIntegralSum(Mat input, int index) {

        Mat sum = new Mat(input.rows() + 1, input.cols() + 1, CV_32F);
        Mat sqsum = new Mat(input.rows() + 1, input.cols() + 1, CV_64F);

        // finds the integral
        Imgproc.integral2(input, sum, sqsum, CV_32F);

        integrals[index] = sum.get(sum.rows()-2, sum.cols() -2)[0];

    }

    /**
     * waits in a loop until all three of the Threads that are calculating integrals are done
     */
    private void waitForThread() {

        while (isNotFinished()) {

            if (lastIntegralString != null) {
                updateTextView(lastIntegralString);
            }

        }

    }

    /**
     *  Determines all threads are done finding the integrals by finding out if the Integrals array has any NULL(-1) values.
     * @return returns FALSE if all threads ARE done and returns TRUE if all threads are NOT done.
     */
    private boolean isNotFinished() {

        boolean isNotFinished = false;

        for (double integral : integrals) {
            if (integral == NULL) {
                isNotFinished = true;
            }
        }
        return isNotFinished;

    }

    /**
     * Resets the Integrals array by updating all values as NULL(-1) this will make isNotFinished() return not finished.
     * Saves the integrals array in lastIntegrals
     */
    private void resetIntegrals() {

        for (int i = 0; i < integrals.length; i++) {
            lastIntegrals[i] = integrals[i];
            integrals[i] = NULL;
        }

    }

    /**
     * Takes the three integral values in the Integrals array and converts them into a String
     */
    private void createString() {

        StringBuilder s = new StringBuilder("\n\n");

        for (int i = 0; i < integrals.length; i++) {
            if (i == LEFT) {
                s.append("Left: ");
            } else if (i == MIDDLE) {
                s.append("Middle: ");
            } else if (i == RIGHT){
                s.append("Right: ");
            }
            s.append(integrals[i]);
            s.append(" ");
        }

        lastIntegralString = s.toString();

    }
//
//    private static String createString(double[] arr) {
//
//        StringBuilder s = new StringBuilder("\n\n");
//
//        for (int i = 0; i < arr.length; i++) {
//            if (i == LEFT) {
//                s.append("Left: ");
//            } else if (i == MIDDLE) {
//                s.append("Middle: ");
//            } else if (i == RIGHT){
//                s.append("Right: ");
//            }
//            s.append(arr[i]);
//            s.append(" ");
//        }
//
//         return s.toString();
//
//    }


    /**
     * returns the index for the largest value in a double array
     * @param vals double array
     * @return index # of the largest value.
     */
    private static int getLargest(double[] vals) {

        double largest = vals[0];
        int index = 0;

        for (int i = index; i < vals.length; i++) {
            if (vals[i] > largest) {
                largest = vals[i];
                index = i;
            }
        }

        return index;

    }

    /**
     * Will split the image into 9 parts. 3 vertical splits and 3 horizontal splits
     * @param input image that will be drawn on
     */
    private static void splitMat(Mat input) {

        // Vertical Vertical
        Imgproc.line(
                input,
                new Point(input.cols()/3, 0),
                new Point(input.cols()/3, input.rows()),
                new Scalar(0, 255, 0),
                4
        );

        Imgproc.line(
                input,
                new Point((input.cols()/3) *2, 0),
                new Point((input.cols()/3) *2, input.rows()),
                new Scalar(0,255,0),
                4
        );

        // Horizontal Lines
        Imgproc.line(
                input,
                new Point(0, ((input.rows()/8)*5)-(input.rows()/BLOCK_HEIGHT)),
                new Point(input.cols(), ((input.rows()/8)*5)-(input.rows()/BLOCK_HEIGHT)),
                new Scalar(0,255,0),
                4);

        Imgproc.line(
                input,
                new Point(0, ((input.rows()/8)*5)+(input.rows()/BLOCK_HEIGHT)),
                new Point(input.cols(), ((input.rows()/8)*5)+(input.rows()/BLOCK_HEIGHT)),
                new Scalar(0,255,0),
                4
        );

    }

    /**
     * Scales image to specified dimensions
     * @param image image to be scaled
     * @return scaled image
     */
    private static Bitmap scaleImage(Bitmap image) {

        int width = image.getWidth();
        int height = image.getHeight();

        int boundingWidth = dpToPx(imgWidth);
        int boundingHeight = dpToPx(imgHeight);

        float xScale = ((float) boundingWidth)/width;
        float yScale = ((float) boundingHeight)/height;

        Matrix matrix = new Matrix();
        matrix.postScale(xScale, yScale);
        return Bitmap.createBitmap(image, 0,0,width,height,matrix,true);

    }

    /**
     * Utility Functions
     */

    private static int dpToPx(int dp) {

        float density = singleton.getActivity().getResources().getDisplayMetrics().density;

        return Math.round((float)dp * density);
    }



    private static Mat bitmapToMat (Bitmap bit, int cvType) {

        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

    /**
     * Prints the string value in the main activity
     * @param string string to be printed
     */
    private void updateTextView(final String string) {

        singleton.getActivity().runOnUiThread(new Runnable() {

            @Override
            public void run() {

                mat.setText(string);

            }

        });
    }

}
