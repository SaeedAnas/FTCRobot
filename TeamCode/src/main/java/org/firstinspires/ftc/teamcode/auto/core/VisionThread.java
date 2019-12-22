package org.firstinspires.ftc.teamcode.auto.core;

import android.graphics.Bitmap;
import android.widget.ImageView;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcontroller.internal.SingletonManager.singleton;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.auto.core.VariableManager.vars;
import org.firstinspires.ftc.teamcode.auto.vision.SkystoneDetectionOpMode;

public class VisionThread implements Runnable {
    private VuforiaLocalizer vuforia = null;
    private static VisionPipeline p;
    private ImageView stream = singleton.getStream();
    private Telemetry telemetry = vars.getTelemetry();
    private static Mat lastFrame;

    VisionThread() {
        initVuforia();
    }

    @Override
    public void run() {
        while(Thread.currentThread().isAlive()) {
            stream();
        }
    }

    private void initVuforia() {
        p = new VisionPipeline();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
    }

    private void stream() {
        VuforiaLocalizer.CloseableFrame vuFrame = null;
        if (!vuforia.getFrameQueue().isEmpty()) {
            try {
                vuFrame = vuforia.getFrameQueue().take();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            if (vuFrame != null) {
                for (int i = 0; i < vuFrame.getNumImages(); i++) {
                    Image img = vuFrame.getImage(i);
                    if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                        Bitmap displayBitmap = processImage(img);
                        update(displayBitmap);
                        telemetry.addData("Index: ", p.getIndex());
                    }
                }
            }
        }
        telemetry.update();

    }

    private Bitmap processImage(Image img) {
        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());
        Mat mat = bitmapToMat(bm, CvType.CV_8UC3);
        Mat ret = p.processFrame(mat);
        lastFrame = ret;
        final Bitmap displayBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(ret, displayBitmap);
        return displayBitmap;
    }

    private void update(final Bitmap displayBitmap) {
        singleton.getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                stream.setImageBitmap(displayBitmap);
            }
        });
    }

    public static int getSkystonePosition() {
        if (lastFrame != null) {
            return p.searchMap(lastFrame);
        }
        return -1;
    }

    private Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }


}
