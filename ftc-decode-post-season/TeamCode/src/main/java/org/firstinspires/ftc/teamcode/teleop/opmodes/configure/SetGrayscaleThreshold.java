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
import org.firstinspires.ftc.teamcode.xml.GrayscaleParametersXMLIn;
import org.firstinspires.ftc.teamcode.xml.GrayscaleParametersXMLOut;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;

// This OpMode gives the drive team a way to check and modify
// the grayscale thresholding of a cropped webcam frame.
@TeleOp(name = "SetGrayscaleThreshold", group = "Configure")
//@Disabled
public class SetGrayscaleThreshold extends LinearOpMode {
    private static final String TAG = SetGrayscaleThreshold.class.getSimpleName();

    private GrayscaleThresholdProcessor grayscaleThresholdProcessor;
    private FTCButton factoryResetButton;
    private FTCButton factoryResetConfirmButton;
    private FTCButton factoryResetCancelButton;
    private boolean factoryResetRequested = false;

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
                try {
                    new GrayscaleParametersXMLOut(imageParameters, grayParameters,
                            WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
                } catch (ParserConfigurationException pe) {
                    throw new AutonomousRobotException(TAG, pe.getMessage());
                }

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

    // Take no action if this method is called before an OpMode is selected -
    // pixelCountRendering will be null!
    private void updateIncreaseThreshold() {
        if (increaseThresholdButton.is(FTCButton.State.TAP)) {
            grayParametersChanged = true;
            grayParameters.threshold_low += THRESHOLD_CHANGE;
            grayscaleThresholdProcessor.setGrayscaleThreshold(grayParameters.threshold_low);

            //!! A screen tap on the DS is also required.
        }
    }

    private void updateDecreaseThreshold() {
        if (decreaseThresholdButton.is(FTCButton.State.TAP)) {
            grayParametersChanged = true;
            grayParameters.threshold_low -= THRESHOLD_CHANGE;
            grayscaleThresholdProcessor.setGrayscaleThreshold(grayParameters.threshold_low);

            //!! A screen tap on the DS is also required.
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
        GrayscaleParametersXMLIn grayscaleParametersXMLIn;
        try {
            grayscaleParametersXMLIn = new GrayscaleParametersXMLIn(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new AutonomousRobotException(TAG, e.getMessage());
        }

        imageParameters = grayscaleParametersXMLIn.getImageParameters();
        grayParameters = grayscaleParametersXMLIn.getGrayscaleParameters();
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
