package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class GrayscaleThresholdProcessor implements VisionProcessor {
    private Mat processedMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize your matrices or variables here
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        // Apply OpenCV transformations here
        // Example: Convert to Grayscale
        Imgproc.cvtColor(frame, processedMat, Imgproc.COLOR_RGB2GRAY);

        //**TODO Need AtomicInteger for the current grayscale threshold.

        return processedMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasToBotPx,
                            Object userContext) {
        // Optional: Draw overlays on the Driver Station viewport using Android Canvas
    }
}
