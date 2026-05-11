package org.firstinspires.ftc.teamcode.auto;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameAccess;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.xml.RobotActionXML;
import org.firstinspires.ftc.teamcode.xml.StartParameters;
import org.firstinspires.ftc.teamcode.xml.StartParametersXMLIn;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

public class FTCAuto {
    private static final String TAG = FTCAuto.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final LinearOpMode linearOpMode;
    private final FTCRobot robot;
    private final String workingDirectory;
    private final StartParametersXMLIn startParametersXML;
    private final StartParameters startParameters;
    private final RobotActionXML.RobotActionData actionData;

    private enum ActionStatus {
        NO_ACTIONS_TAKEN, OPMODE_INACTIVE,
        ACTION_COMPLETE, ALL_ACTIONS_COMPLETE,
        ACTION_STOP, ACTION_STOP_ALL
    }

    // Webcams
    private final EnumSet<RobotConstantsDecode.InternalWebcamId> openWebcams = EnumSet.noneOf(RobotConstantsDecode.InternalWebcamId.class);

    // Main class for the autonomous run.
    public FTCAuto(RobotConstants.Alliance pAlliance, LinearOpMode pLinearOpMode, FTCRobot pRobot,
                   RobotConstantsDecode.OpMode pOpMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException {

        RobotLogCommon.c(TAG, "Constructing FTCAuto with alliance " + pAlliance + " running OpMode " + pOpMode);

        alliance = pAlliance;
        linearOpMode = pLinearOpMode; // FTC context
        robot = pRobot; // robot hardware

        //**TODO Keep this here but commented out until we
        // track down all "stuck in stop" conditions.
        // Set up an AutonomousTimer as a failsafe to prevent
        // "stuck in stop". Start the timer in runRobot below.

        // Get the directory for the various configuration files.
        workingDirectory = WorkingDirectory.getWorkingDirectory();

        // Raed the start parameters from the XML file.
        startParametersXML = new StartParametersXMLIn(workingDirectory + RobotConstants.XML_DIR);
        startParameters = startParametersXML.getStartParameters();

        // Read the RobotAction XXX.xml file for all OpModes.
        RobotLogCommon.c(TAG, "Getting the Autonomous choreographies from " + robot.startParameters.robotActionFilename);
        RobotActionXML actionXML = new RobotActionXML(workingDirectory + RobotConstants.XML_DIR + robot.startParameters.robotActionFilename);

        // Extract data from the parsed XML file for the selected OpMode only.
        actionData = actionXML.getOpModeData(pOpMode.toString());

        RobotLogCommon.CommonLogLevel logLevel = actionData.logLevel;
        RobotLogCommon.setMostDetailedLogLevel(logLevel);
        RobotLogCommon.c(TAG, "Most detailed log level " + RobotLogCommon.getMostDetailedLogLevel());

        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            // For competition OpModes start the front webcam here so that webcam
            // initialization can take place before Autonomous starts. For DECODE
            // start the front camera with two processors: one for AprilTags (enabled)
            // and one for raw frames (disabled).
            // TEST OpModes can start any camera synchronously by using the <START_CAMERA>
            // action in RobotAction.xml
            if (pOpMode.getOpModeType() != RobotConstantsDecode.OpMode.OpModeType.AUTO_TEST) {
                VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                        robot.configuredWebcams.get(RobotConstantsDecode.InternalWebcamId.FRONT_WEBCAM);
                if (frontWebcamConfiguration != null) {
                    // Configure the front webcam for both AprilTags and raw frames.
                    EnumMap<RobotConstantsDecode.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors =
                            new EnumMap<>(RobotConstantsDecode.ProcessorIdentifier.class);

                    VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                            // Follow the MultiPortal sample, which only includes setLensIntrinsics
                            .setLensIntrinsics(frontWebcamConfiguration.cameraCalibration.focalLengthX,
                                    frontWebcamConfiguration.cameraCalibration.focalLengthY,
                                    frontWebcamConfiguration.cameraCalibration.principalPointX,
                                    frontWebcamConfiguration.cameraCalibration.principalPointY)
                            .build();

                    // Initialize the AprilTag processor as enabled.
                    assignedProcessors.put(RobotConstantsDecode.ProcessorIdentifier.APRIL_TAG, Pair.create(aprilTagProcessor, true));

                    // Initialize the raw frame processor as disabled; explicitly enable when needed.
                    VisionProcessor rawFrameProcessor = new RawFrameProcessor.Builder().build();
                    assignedProcessors.put(RobotConstantsDecode.ProcessorIdentifier.RAW_FRAME, Pair.create(rawFrameProcessor, false));

                    VisionPortalWebcam visionPortalWebcam = new VisionPortalWebcam(frontWebcamConfiguration, assignedProcessors);
                    frontWebcamConfiguration.setVisionPortalWebcam(visionPortalWebcam);
                    if (!visionPortalWebcam.waitForWebcamStart(2000))
                        throw new AutonomousRobotException(TAG, "Unable to start front webcam");

                    openWebcams.add(RobotConstantsDecode.InternalWebcamId.FRONT_WEBCAM);
                }
            }
        }

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws Exception {
        // In order for finally() to run we need to have a catch block below.
        // Without it the FTC runtime, which must have an UncaughtExceptionHandler,
        // prevents our finally() from running.
        ActionStatus actionStatus = ActionStatus.NO_ACTIONS_TAKEN; // default
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs.
            // 12/28/2022 From the FTC SDK documentation: "whether the OpMode is
            // currently active. If this returns false, you should break out of
            // the loop in your runOpMode() method and return to its caller".
            //&& 1/3/2024 Not clear that this condition has ever occurred.
            // But leave the check here and watch the logs.
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");
                //## throwing an AutonomousRobotException here will result in
                //## a "stuck in stop" condition and the FTC runtime will restart
                // the robot.
                RobotLog.dd(TAG, "OpMode unexpectedly inactive at the start of runRobot()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive at the start of runRobot()");
                return;
            }

            RobotLogCommon.i(TAG, "FTCAuto runRobot()");

            // Follow the choreography specified in the robot action file.
            List<RobotXMLElement> actions = actionData.actions;
            actionStatus = executeActions(actions);
        } finally {
            //**TODO 10/23/2025 Need to revisit this: tests of the AutonomousTimer
            // show that the finally block is entered even after
            // LinearOpMode.requestOpModeStop() has been called.
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "FTCAuto OpMode not active in finally block");
                RobotLogCommon.i(TAG, "FTCAuto OpMode not active in finally block");
            } else {
                RobotLog.dd(TAG, "In FTCAuto finally block");
                RobotLogCommon.i(TAG, "In FTCAuto finally block");

                // Log the result of the topmost set of actions.
                RobotLog.dd(TAG, "Final action status " + actionStatus);

                // Close any configured webcams.
                if (robot.configuredWebcams != null) { // if webcam(s) are configured in
                    RobotLogCommon.i(TAG, "In FTCAuto finally: close webcam(s)");
                    robot.configuredWebcams.forEach((k, v) -> {
                        if (v != null && v.getVisionPortalWebcam() != null)
                            v.getVisionPortalWebcam().finalShutdown();
                    });
                }
            }
        }

        RobotLogCommon.i(TAG, "Exiting FTCAuto");
        linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
        linearOpMode.telemetry.update();
    }

    //===============================================================================================
    //===============================================================================================

    private ActionStatus executeActions(List<RobotXMLElement> pActions) throws Exception {
        for (RobotXMLElement action : pActions) {
            if (!linearOpMode.opModeIsActive()) {
                RobotLog.dd(TAG, "OpMode went inactive in the main Autonomous loop");
                RobotLogCommon.d(TAG, "OpMode went inactive in the main Autonomous loop");
                return ActionStatus.OPMODE_INACTIVE;
            }

            ActionStatus actionStatus = executeAction(action);
            if (actionStatus == ActionStatus.ACTION_STOP || actionStatus == ActionStatus.ACTION_STOP_ALL)
                return actionStatus;
        }

        return ActionStatus.ALL_ACTIONS_COMPLETE;
    }

    // Using the XML elements and attributes from the configuration file
    // RobotAction.xml, execute a single action.
    // Note that executeAction may return a value other than ActionStatus.ACTION_COMPLETE
    // as a signal to stop or short-circuit the current set of actions.
    @SuppressLint("DefaultLocale")
    private ActionStatus executeAction(RobotXMLElement pAction) throws Exception {
        // Set up XPath access to the current action.
        XPathAccess actionXPath = new XPathAccess(pAction);
        String actionName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto action " + actionName);

        switch (actionName) {

            case "START_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.add(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to start webcam " + webcamId + " but it is already open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                // The <START_WEBCAM> in RobotAction.xml may contain a single
                // <processor> element or a <processor_set> element with
                // multiple <processor> children.
                ArrayList<Pair<RobotConstantsDecode.ProcessorIdentifier, Boolean>> requestedProcessors;
                String processorIdString = actionXPath.getText("processor", "NPOS").toUpperCase();
                if (!processorIdString.equals("NPOS")) { // single processor id
                    RobotConstantsDecode.ProcessorIdentifier processorId = RobotConstantsDecode.ProcessorIdentifier.valueOf(processorIdString);
                    requestedProcessors = new ArrayList<Pair<RobotConstantsDecode.ProcessorIdentifier, Boolean>>() {{
                        add(Pair.create(processorId, true)); // default to enable on webcam start
                    }};
                } else // multiple processors, i.e. a <processor_set>.
                    requestedProcessors = RobotActionXML.getStartWebcamProcessors(pAction);

                // The collection requestedProcessors contains the ids of one or
                // more processors to be assigned to the webcam. Check that each
                // processor id is present in the webcam's <processor_set> in
                // RobotConfig.xml and create the actual VisionProcessor objects.
                EnumMap<RobotConstantsDecode.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors =
                        new EnumMap<>(RobotConstantsDecode.ProcessorIdentifier.class);
                ArrayList<RobotConstantsDecode.ProcessorIdentifier> processorIdentifiers = configuredWebcam.processorIdentifiers;
                for (Pair<RobotConstantsDecode.ProcessorIdentifier, Boolean> entry : requestedProcessors) {
                    if (!processorIdentifiers.contains(entry.first))
                        throw new AutonomousRobotException(TAG, "Assigned processor with id " + entry.first + " is not in the configuration for webcam " + configuredWebcam.internalWebcamId);

                    switch (entry.first) {
                        case RAW_FRAME: {
                            VisionProcessor rawFrameProcessor = new RawFrameProcessor.Builder().build();
                            assignedProcessors.put(RobotConstantsDecode.ProcessorIdentifier.RAW_FRAME,
                                    Pair.create(rawFrameProcessor, entry.second));
                            break;
                        }
                        case APRIL_TAG: {
                            VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                                    // Follow the MultiPortal sample, which only includes setLensIntrinsics
                                    .setLensIntrinsics(configuredWebcam.cameraCalibration.focalLengthX,
                                            configuredWebcam.cameraCalibration.focalLengthY,
                                            configuredWebcam.cameraCalibration.principalPointX,
                                            configuredWebcam.cameraCalibration.principalPointY)
                                    .build();

                            assignedProcessors.put(RobotConstantsDecode.ProcessorIdentifier.APRIL_TAG,
                                    Pair.create(aprilTagProcessor, entry.second));
                            break;
                        }

                        default:
                            throw new AutonomousRobotException(TAG, "Invalid processor id " + entry.first);
                    }
                }

                // Now actually create the VisionPortalWebcam with the active
                // (but not necessarily enabled) processors.
                VisionPortalWebcam visionPortalWebcam = new VisionPortalWebcam(configuredWebcam, assignedProcessors);
                configuredWebcam.setVisionPortalWebcam(visionPortalWebcam);
                openWebcams.add(webcamId);

                break;
            }

            case "WAIT_FOR_WEBCAM_START": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to wait for the startup of webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                int timeout = actionXPath.getRequiredInt("timeout_ms");
                if (!configuredWebcam.getVisionPortalWebcam().waitForWebcamStart(timeout)) {
                    return ActionStatus.ACTION_STOP_ALL; // no webcam, just give up
                }

                break;
            }

            case "STOP_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to close webcam " + webcamId + " but it is not open");

                VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcam =
                        robot.configuredWebcams.get(webcamId);
                if (configuredWebcam == null)
                    throw new AutonomousRobotException(TAG, "Attempt to start a webcam that is not in the configuration " + webcamId);

                configuredWebcam.getVisionPortalWebcam().finalShutdown();
                configuredWebcam.setVisionPortalWebcam(null);
                openWebcams.remove(webcamId);
                RobotLogCommon.d(TAG, "Stopped webcam " + webcamIdString);
                break;
            }

            case "STOP_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to stop streaming webcam " + webcamId + " but it is not open");

                webcam.getVisionPortalWebcam().stopStreaming();
                RobotLogCommon.d(TAG, "Stopped streaming webcam " + webcamIdString);
                break;
            }

            case "RESUME_WEBCAM_STREAMING": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to resume streaming webcam " + webcamId + " but it is not open");

                int timeout = actionXPath.getRequiredInt("timeout_ms");
                webcam.getVisionPortalWebcam().resumeStreaming(timeout);
                RobotLogCommon.d(TAG, "Resumed streaming webcam " + webcamIdString);
                break;
            }

            case "ENABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to enable processor on webcam " + webcamId + " but it is not open");

                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsDecode.ProcessorIdentifier processorId = RobotConstantsDecode.ProcessorIdentifier.valueOf(processorIdString);

                webcam.getVisionPortalWebcam().enableProcessor(processorId);
                RobotLogCommon.d(TAG, "Enabled processor on webcam " + webcamIdString);
                break;
            }

            case "DISABLE_PROCESSOR": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to disable processor on webcam " + webcamId + " but it is not open");

                String processorIdString = actionXPath.getRequiredText("processor").toUpperCase();
                RobotConstantsDecode.ProcessorIdentifier processorId = RobotConstantsDecode.ProcessorIdentifier.valueOf(processorIdString);

                webcam.getVisionPortalWebcam().disableProcessor(processorId);
                RobotLogCommon.d(TAG, "Disabled processor on webcam " + webcamIdString);
                break;
            }

            // For testing, get a frame from a webcam managed by the VisionPortal
            // API and write it out to a file. Assume that the webcam has already
            // been started.
            case "TAKE_PICTURE_WEBCAM": {
                String webcamIdString = actionXPath.getRequiredText("internal_webcam_id").toUpperCase();
                RobotConstantsDecode.InternalWebcamId webcamId =
                        RobotConstantsDecode.InternalWebcamId.valueOf(webcamIdString);
                VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                        Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

                if (!openWebcams.contains(webcamId))
                    throw new AutonomousRobotException(TAG, "Attempt to take picture on webcam " + webcamId + " but it is not open");

                VisionProcessor rawFrameProcessor =
                        webcam.getVisionPortalWebcam().getEnabledProcessor(RobotConstantsDecode.ProcessorIdentifier.RAW_FRAME);
                if (rawFrameProcessor == null)
                    throw new AutonomousRobotException(TAG, "The RAW_FRAME processor is not active");

                RawFrameAccess rawFrameAccess = new RawFrameAccess((RawFrameProcessor) rawFrameProcessor);
                Pair<Mat, Date> image = rawFrameAccess.getImage();
                if (image == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from " + webcamIdString);
                    linearOpMode.telemetry.update();
                } else {
                    RobotLogCommon.d(TAG, "Took a picture with " + webcamIdString);
                    String fileDate = TimeStamp.getDateTimeStamp(image.second);
                    String outputFilenamePreamble = workingDirectory + RobotConstants.IMAGE_DIR + webcamIdString + "_" + fileDate;

                    String imageFilename = outputFilenamePreamble + "_IMG.png";
                    RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                    Imgcodecs.imwrite(imageFilename, image.first);

                    RobotLogCommon.d(TAG, "Image width " + image.first.cols() + ", height " + image.first.rows());
                    linearOpMode.telemetry.addData("Take picture with webcam:", "successful");
                    linearOpMode.telemetry.update();
                }

                break;
            }

            // Common actions.
            case "SLEEP": { // I want sleep :)
                int sleepMs = actionXPath.getRequiredInt("ms");
                sleepInLoop(sleepMs);
                break;
            }

            // In testing this gives us a way to short-circuit a set
            //  of actions without commenting out any XML.
            case "STOP": {
                sleep(500);
                return ActionStatus.ACTION_STOP;
            }

            // In testing this gives us a way to short-circuit all
            // actions without commenting out any XML.
            case "STOP_ALL": {
                sleep(500);
                return ActionStatus.ACTION_STOP_ALL;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the action " + actionName);
            }
        }
        // END of switch for actions from RobotAction.xml.

        // Action completed normally.
        return ActionStatus.ACTION_COMPLETE;
    }


    // BELOW: methods called from case labels inside the switch for actions from RobotAction.xml.

    // Sleeps but also tests if the OpMode is still active.
    // Telemetry should keep the FTC runtime from shutting us down due to inactivity.
    private void sleepInLoop(int pMilliseconds) {
        RobotLogCommon.d(TAG, "Sleep for " + pMilliseconds + " milliseconds");

        int numSleeps = pMilliseconds / 100;
        int sleepRemainder = pMilliseconds % 100;
        for (int i = 0; i < numSleeps; i++) {
            if (!linearOpMode.opModeIsActive())
                return;
            linearOpMode.sleep(100);
        }

        if (linearOpMode.opModeIsActive() && sleepRemainder != 0)
            linearOpMode.sleep(sleepRemainder);
    }


}

