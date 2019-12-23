package org.firstinspires.ftc.teamcode.auto.vision;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.ImageView;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcontroller.internal.SingletonManager.singleton;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.VUFORIA_KEY;

public class VisionThread implements Runnable {
    private VuforiaLocalizer vuforia = null;
    private VisionPipeline p;
    private ImageView stream = singleton.getStream();


    public VisionThread() {
        initVuforia();
    }

    @Override
    public void run() {
        while(!Thread.currentThread().isInterrupted()) {
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
                        Bitmap displayBitmap = p.processImage(img);
                        update(displayBitmap);
                    }
                }
            }
        }

    }



    private void update(final Bitmap displayBitmap) {
        singleton.getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                stream.setImageBitmap(displayBitmap);
            }
        });
    }

}
