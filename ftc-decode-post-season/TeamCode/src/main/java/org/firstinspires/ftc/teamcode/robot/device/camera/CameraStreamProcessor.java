package org.firstinspires.ftc.teamcode.robot.device.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

//##PY Modified to align with the changes in the RawFrameProcessor.
public class CameraStreamProcessor implements VisionProcessor {

    private final String TAG = CameraStreamProcessor.class.getSimpleName();

    private final AtomicReference<CameraStreamRendering> cameraStreamRenderingRef = new AtomicReference<>();

    //## This is a callback. It definitely runs on another thread.
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Init is called from VisionPortalImpl when the first frame for this
        // processor has been received; the frame itself is not passed in
        // here.
    }

    //## This is a callback; assume it's running on another thread.
    //## The returned Object is passed in to onDrawFrame as
    // Object userContext.
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        return input;
    }

    //## This is a callback.
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof Mat))
            throw new AutonomousRobotException(TAG, "CameraStreamProcessorImpl.onDrawFrame expected OpenCV Mat"); // somebody changed processFrame.

        CameraStreamRendering cameraStreamRendering = cameraStreamRenderingRef.get();
        if (cameraStreamRendering != null) // has rendering been set?
            cameraStreamRendering.renderFrameToCanvas((Mat) userContext, canvas, onscreenWidth, onscreenHeight);
    }

    public void setCameraStreamRendering(CameraStreamRendering pRendering) {
        cameraStreamRenderingRef.set(pRendering);
    }

}

