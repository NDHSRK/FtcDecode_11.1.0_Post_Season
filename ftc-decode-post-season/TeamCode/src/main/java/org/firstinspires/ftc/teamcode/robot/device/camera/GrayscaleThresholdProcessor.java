package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class GrayscaleThresholdProcessor implements VisionProcessor {
    private final VisionParameters.ImageParameters imageParameters;
    private final String outputFilePreamble;
    private final AtomicBoolean requestWriteToFile = new AtomicBoolean();
    private int writeToFileCount;

    private final AtomicInteger currentThresholdLow = new AtomicInteger(128);
    private Mat processedMat = new Mat();

    public GrayscaleThresholdProcessor(VisionParameters.ImageParameters pImageParameters) {
        imageParameters = pImageParameters;
        outputFilePreamble = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
    }

    //## This is a callback. It definitely runs on another thread.
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Init is called from VisionPortalImpl when the first frame for
        // this processor has been received; the frame itself is not passed
        // in here.
    }

    //## This is a callback; assume it's running on another thread.
    //## The returned Object is passed in to onDrawFrame as
    // Object userContext.
    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        boolean writeToFileNow = requestWriteToFile.getAndSet(false);
        if (writeToFileNow)
            writeToFileCount++;

        // From the EasyOpenCV readme:
        //!!IMPORTANT NOTE: EasyOpenCV delivers RGBA frames
        // So we need to convert to GRAY or BGR for OpenCV here.
        Imgproc.cvtColor(frame, processedMat, Imgproc.COLOR_RGBA2GRAY);

        // Apply OpenCV transformations here, in this case a simple grayscale threshold.
        int thresholdLow = currentThresholdLow.get();
        processedMat = ImageUtils.preProcessImage(processedMat, imageParameters);
        Imgproc.threshold(processedMat, processedMat,
                Math.abs(thresholdLow), // threshold value
                255,   // white
                thresholdLow >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type

        if (writeToFileNow) {
            String outputFilename = outputFilePreamble + "GrayThr_" + String.format(Locale.US, "_%04d_THR.png", writeToFileCount);
            Imgcodecs.imwrite(outputFilename, processedMat);
        }

        return processedMat;
    }

    //## This is a callback.
    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasToBotPx,
                            Object userContext) {
        // Optional: Draw overlays on the Driver Station viewport using Android Canvas
    }

    public void setGrayscaleThreshold(int pThresholdLow) {
        currentThresholdLow.set(pThresholdLow);
    }

    public void requestWriteToFile() {
        requestWriteToFile.set(true);
    }
}
