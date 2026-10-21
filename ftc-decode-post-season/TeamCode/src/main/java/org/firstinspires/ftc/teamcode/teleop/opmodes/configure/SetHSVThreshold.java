package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.GrayscaleThresholdProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
//**TODO Implement these classes when you are reading in and writing out
// an entire XML file. See Decode 11.1.0 StartParametersXMLIn/Out.
// Otherwise put element replacement methods into the main XML file,
// e.g. BiobuzzParametersXML.java in IJOpenCVTestbed4.
//import org.firstinspires.ftc.teamcode.xml.GrayscaleParametersXMLIn;
//import org.firstinspires.ftc.teamcode.xml.GrayscaleParametersXMLOut;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;

import java.util.Objects;

// This OpMode gives the drive team a way to check and modify
// the HSV inRange thresholding of a cropped webcam frame.

//**TODO !! 10/20/26 Copied/renamed from SetGrayscaleThreshold.

@TeleOp(name = "SetHSVThreshold", group = "Configure")
//@Disabled
public class SetHSVThreshold extends LinearOpMode {
    private static final String TAG = SetHSVThreshold.class.getSimpleName();

    private GrayscaleThresholdProcessor grayscaleThresholdProcessor;
    private FTCButton factoryResetButton;
    private FTCButton factoryResetConfirmButton;
    private FTCButton factoryResetCancelButton;
    private boolean factoryResetRequested = false;

    private int threshholdLow; //**TODO initialize hue low, high; saturation
    // low; value lowfrom XML

    //**TODO Oh no! Also need buttons for factoryResetButton, factoryResetConfirmButton,
    // factoryResetCancelButton, writeThresholdedImageToFileButton
    /*
        factoryResetButton = new FTCButton(() -> gamepad1.start);
        factoryResetConfirmButton = new FTCButton(() -> gamepad1.y);
        factoryResetCancelButton = new FTCButton(() -> gamepad1.x);

        increaseThresholdButton = new FTCButton(() -> gamepad1.dpad_up);
        decreaseThresholdButton = new FTCButton(() -> gamepad1.dpad_down);
        writeThresholdedImageToFileButton = new FTCButton(() -> gamepad1.left_bumper);
     */

    //**TODO Buttons for HSV color calibration
    /*
    Toggle for red, blue, yellow -> DP up
    Hue low: up and down -> B, A
    Hue high: up and down -> Y, X
    Sat low: up and down -> left bumper, left trigger as button
    Val low: up and down -> right bumper, right trigger as button

    Need a button for
    requestImageCapture = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);
    use DPAD down
    A button press issues a call to pixelCountRendering.requestImageCapture()
    See C:\Users\lonep\OneDrive\Documents\FTC\FTC 2026 Post Season\AutomaticThresholding\teleop configure from CenterStage
    and C:\Users\lonep\OneDrive\Documents\FTC\FTC 2026 Post Season\AutomaticThresholding\robot camera from CenterStage
    class PixelCountRendering implements CameraStreamRendering gets an image and performs
    OpenCV operations on it.
    */

    private FTCButton increaseThresholdButton;
    private FTCButton decreaseThresholdButton;
    private FTCButton writeThresholdedImageToFileButton;

    private static final int THRESHOLD_CHANGE = 5;
    private VisionParameters.ImageParameters imageParameters;
    private VisionParameters.GrayParameters grayParameters;
    private boolean grayParametersChanged = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotLogCommon.c(TAG, "Initializing SetGrayscaleThreshold");

        // Read in the grayscale parameters from GrayscaleParameters.xml and
        // set class variables.
        initializeGrayscaleParameters();

        // Get the camera configuration from RobotConfig.xml.
        FTCRobot robot = new FTCRobot(this, RobotConstants.RunType.TELEOP_VISION_PREVIEW);

        // Start the front webcam with the grayscale threshold processor.
        if (robot.configuredWebcams == null)
            throw new AutonomousRobotException(TAG, "There are no webcams in the current configuration");

        VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                Objects.requireNonNull(robot.configuredWebcams.get(RobotConstantsDecode.InternalWebcamId.FRONT_WEBCAM),
                        TAG + " The FRONT_WEBCAM is not configured");

        grayscaleThresholdProcessor = new GrayscaleThresholdProcessor(imageParameters);
        VisionPortalWebcam grayscaleThresholdWebcam = new VisionPortalWebcam(frontWebcamConfiguration,
                RobotConstantsDecode.ProcessorIdentifier.CAMERA_STREAM_PREVIEW,
                Pair.create(grayscaleThresholdProcessor, true));

        if (!grayscaleThresholdWebcam.waitForWebcamStart(2000))
            throw new AutonomousRobotException(TAG, "Grayscale threshold webcam timed out on start");

        frontWebcamConfiguration.setVisionPortalWebcam(grayscaleThresholdWebcam);
        RobotLogCommon.c(TAG, "SetGrayscaleThreshold successfully started on the front webcam");

        factoryResetButton = new FTCButton(() -> gamepad1.start);
        factoryResetConfirmButton = new FTCButton(() -> gamepad1.y);
        factoryResetCancelButton = new FTCButton(() -> gamepad1.x);

        increaseThresholdButton = new FTCButton(() -> gamepad1.dpad_up);
        decreaseThresholdButton = new FTCButton(() -> gamepad1.dpad_down);
        writeThresholdedImageToFileButton = new FTCButton(() -> gamepad1.left_bumper);

        while (!isStarted() && !isStopRequested()) {
            updateTelemetry();
            updateButtons();
            updateActions();
        }

        // The driver has hit the play button.
        if (opModeIsActive()) {
            if (grayParametersChanged) {
//**TODO                try {
//                    new GrayscaleParametersXMLOut(imageParameters, grayParameters,
//                            WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
//                } catch (ParserConfigurationException pe) {
//                    throw new AutonomousRobotException(TAG, pe.getMessage());
//                }

                RobotLogCommon.i(TAG, "Writing GrayscaleParameters.xml");
                telemetry.addLine("Writing GrayscaleParameters.xml");
            } else {
                RobotLogCommon.i(TAG, "No changes to GrayscaleParameters.xml");
                telemetry.addLine("No changes to GrayscaleParameters.xml");
            }
        }

        telemetry.update();
        sleep(1500);
    }

    private void updateButtons() {
        // If a factory reset has been requested only update
        // the buttons for confirm and cancel.
        if (factoryResetRequested) {
            factoryResetConfirmButton.update();
            factoryResetCancelButton.update();
            return;
        }

        factoryResetButton.update();
        increaseThresholdButton.update();
        decreaseThresholdButton.update();
        writeThresholdedImageToFileButton.update();
    }

    private void updateActions() {
        if (factoryResetRequested) {
            if (!updateFactoryResetConfirm())
                updateFactoryResetCancel();
            return;
        } else
            updateFactoryReset();

        updateIncreaseThreshold();
        updateDecreaseThreshold();
        updateWriteThresholdedImageToFile();
    }

    private void updateFactoryReset() {
        if (factoryResetButton.is(FTCButton.State.TAP)) {
            factoryResetRequested = true;
        }
    }

    private boolean updateFactoryResetConfirm() {
        if (!factoryResetConfirmButton.is(FTCButton.State.TAP))
            return false;

        factoryResetRequested = false;
        initializeGrayscaleParameters();

        RobotLogCommon.i(TAG, "Factory reset of grayscale parameters");
        telemetry.addLine("Factory reset of grayscale parameters");
        telemetry.update();
        sleep(1500);

        return true;
    }

    private void updateFactoryResetCancel() {
        if (factoryResetCancelButton.is(FTCButton.State.TAP))
            factoryResetRequested = false;
    }

    private void updateIncreaseThreshold() {
        if (increaseThresholdButton.is(FTCButton.State.TAP)) {
            grayParametersChanged = true;
            threshholdLow += THRESHOLD_CHANGE;
            grayscaleThresholdProcessor.setGrayscaleThreshold(grayParameters.threshold_low);

            //!! You also need to open a camera stream on the DS.
        }
    }

    private void updateDecreaseThreshold() {
        if (decreaseThresholdButton.is(FTCButton.State.TAP)) {
            grayParametersChanged = true;
            threshholdLow -= THRESHOLD_CHANGE;
            grayscaleThresholdProcessor.setGrayscaleThreshold(grayParameters.threshold_low);

            //!! You also need to open a camera stream on the DS.
        }
    }

    private void updateWriteThresholdedImageToFile() {
        if (writeThresholdedImageToFileButton.is(FTCButton.State.TAP)) {
            grayscaleThresholdProcessor.requestWriteToFile();
        }
    }

    private void initializeGrayscaleParameters() {
        grayParametersChanged = false;

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
//**TODO        GrayscaleParametersXMLIn grayscaleParametersXMLIn;
//        try {
//            grayscaleParametersXMLIn = new GrayscaleParametersXMLIn(fullXMLDir);
//        } catch (ParserConfigurationException | SAXException | IOException e) {
//            throw new AutonomousRobotException(TAG, e.getMessage());
//        }
//
//        imageParameters = grayscaleParametersXMLIn.getImageParameters();
//        grayParameters = grayscaleParametersXMLIn.getGrayscaleParameters();
    }

    private void updateTelemetry() {

        // If a factory reset has been requested only show the confirm/cancel options.
        if (factoryResetRequested) {
            telemetry.addLine("Factory reset requested");
            telemetry.addLine("  Press Y to confirm, X to cancel");
            telemetry.update();
            return;
        }

        telemetry.addLine("The current grayscale threshold is " + grayParameters.threshold_low);
        telemetry.addLine("Press DPAD_UP to increase; DPAD_DOWN to decrease");

        telemetry.addLine("Press START for factory reset");
        telemetry.addLine("Touch play to SAVE changes and END the OpMode");
        telemetry.update();
    }
}
