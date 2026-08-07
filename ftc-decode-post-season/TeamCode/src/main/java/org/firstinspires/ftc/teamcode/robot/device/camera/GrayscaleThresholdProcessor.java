package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.vision.ImageUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
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
    // To display an OpenCV thresholded image on an FTC Driver Station,
    // implement a custom VisionProcessor, override onDrawFrame to convert
    // your OpenCV Mat to an Android Bitmap and use canvas.drawBitmap() so
    // it renders on the canvas that is sent to the Driver Station.
    @Override
    public void onDrawFrame(Canvas pCanvas, int onScreenWidth, int onScreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasToBotPx,
                            Object userContext) {

        Mat thresholded = (Mat) userContext;

        //**TODO Look at the advice on converting back to Imgproc.COLOR_GRAY2RGBA
        // in FrameManagermentAndScaling.java.

        // From CenterStage PixelCountRendering.java but modified after
        // a multi-step exchange with the Google AI Assistant.
        // Show the thresholded ROI in the DS camera stream.
        // First convert the thresholded ROI to an Android Bitmap.
        Bitmap bmp = Bitmap.createBitmap(thresholded.cols(), thresholded.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(thresholded, bmp);

        android.graphics.Rect destRect = new android.graphics.Rect(0, 0, onScreenWidth, onScreenHeight);
        pCanvas.drawBitmap(bmp, null, destRect, null);

        //!! You also need to open a camera stream on the DS.
    }

    public void setGrayscaleThreshold(int pThresholdLow) {
        currentThresholdLow.set(pThresholdLow);
    }

    public void requestWriteToFile() {
        requestWriteToFile.set(true);
    }
}
