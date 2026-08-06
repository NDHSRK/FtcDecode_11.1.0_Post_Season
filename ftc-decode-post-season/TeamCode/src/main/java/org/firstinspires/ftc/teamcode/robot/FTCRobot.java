package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.teamcode.robot.device.motor.TeleOpDriveTrain;
import org.firstinspires.ftc.teamcode.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.xml.StartParameters;
import org.firstinspires.ftc.teamcode.xml.StartParametersXMLIn;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.EnumMap;
import java.util.Optional;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class FTCRobot {

    private static final String TAG = FTCRobot.class.getSimpleName();

    // All motors on the robot for this year's game.
    public enum MotorId {
        LEFT_FRONT_DRIVE, RIGHT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_BACK_DRIVE,
        MOTOR_ID_NPOS // for error checking
    }

    // XML element names in RobotConfig.xml
    // Drive train, TeleOp settings
    public static final String DRIVE_TRAIN_ELEMENT_NAME = "DRIVE_TRAIN";
    public static final String TELEOP_SETTINGS_ELEMENT_NAME = "TELEOP_SETTINGS";

    // Cameras
    public static final String VISION_PORTAL_WEBCAM_ELEMENT_NAME = "VISION_PORTAL_WEBCAM";

    private final HardwareMap hardwareMap;
    public final StartParameters startParameters;
    public final TeleOpDriveTrain teleOpDriveTrain;
    public final TeleOpSettings teleOpSettings;

    public final EnumMap<RobotConstantsDecode.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams;

    public FTCRobot(LinearOpMode pLinearOpMode, RobotConstants.RunType pRunType) throws InterruptedException {
        hardwareMap = pLinearOpMode.hardwareMap;

        RobotLogCommon.c(TAG, "FTCRobot constructor");

        //!! WARNING
        // From the FTC example ConceptMotorBulkRead.java
        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors. [See DriveTrainCore.java]
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.

        //!! 11/16/21 commented out AND NEEDS TO REMAIN OUT because of unpredictable
        // behavior with RUN_TO_POSITION, e.g. immediate reporting of completion of RTP
        // with ending encoder counts of 0 - even though the movement actually took
        // place.
        /*
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        */

        // Get the hardware configuration parameters from RobotConfig XXX.xml.
        try {
            // Get the startup parameters (including the exact file name of
            // RobotConfig XXX.xml).
            String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
            StartParametersXMLIn startParametersXML = new StartParametersXMLIn(xmlDirectory);
            startParameters = startParametersXML.getStartParameters();
            RobotLogCommon.c(TAG, "Configuring the robot from " + startParameters.robotConfigFilename);

            RobotConfigXML configXML = new RobotConfigXML(xmlDirectory + startParameters.robotConfigFilename);
            XPathAccess configXPath;

            // If a drive train is defined and we're running an Autonomous OpMode,
            // which implies that Roadrunner is controlling the drive train, then
            // defer the initialization of the drive train to FTCAuto. Roadrunner
            // needs to know the starting pose of the robot, which we get from
            // RobotAction.xml after the OpMode is selected.
            configXPath = configXML.getPath(DRIVE_TRAIN_ELEMENT_NAME);
            String driveTrainYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
            RobotLogCommon.c(TAG, "Drive train configuration option: " + driveTrainYesNo);
            if (driveTrainYesNo.equals("yes")) {
                // If we're running a TeleOp OpMode, get the TeleOp settings.
                if (pRunType == RobotConstants.RunType.TELEOP || pRunType == RobotConstants.RunType.TELEOP_VISION_PREVIEW) {
                    teleOpSettings = configXML.getTeleOpSettings();
                    RobotLogCommon.c(TAG, "TeleOp configuration: log level " + teleOpSettings.logLevel);
                    RobotLogCommon.setMostDetailedLogLevel(teleOpSettings.logLevel);

                    teleOpDriveTrain = new TeleOpDriveTrain(hardwareMap, configXPath, DRIVE_TRAIN_ELEMENT_NAME);
                 } else { // RunType must be AUTONOMOUS (with Roadrunner)
                    if (pRunType != RobotConstants.RunType.AUTONOMOUS)
                        throw new AutonomousRobotException(TAG, "RunType of AUTONOMOUS is required");

                    teleOpDriveTrain = null;
                    teleOpSettings = null;
                }
            } else {
                teleOpDriveTrain = null;
                teleOpSettings = null;
            }

            if (pRunType == RobotConstants.RunType.TELEOP) {
                configuredWebcams = null;
            } else {
                // Any configured VisionPortal webcams?
                EnumMap<RobotConstantsDecode.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcamsLocal;
                configXPath = configXML.getPath(VISION_PORTAL_WEBCAM_ELEMENT_NAME);
                String webcamYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
                RobotLogCommon.c(TAG, "VisionPortal webcam configuration option: " + webcamYesNo);

                if (webcamYesNo.equals("yes")) {
                    configuredWebcamsLocal = configXML.getConfiguredWebcams();
                    RobotLogCommon.d(TAG, "Number of webcams configured " + configuredWebcamsLocal.size());
                    if (configuredWebcamsLocal.size() > 2)
                        throw new AutonomousRobotException(TAG, "Only two webcams at most are supported");

                    matchHardwareWebcamsWithConfiguredWebcams(configuredWebcamsLocal);
                } else {
                    configuredWebcamsLocal = null;
                    if (pRunType == RobotConstants.RunType.TELEOP_VISION_PREVIEW)
                        throw new AutonomousRobotException(TAG, "RunType of TELEOP_VISION_PREVIEW requires a webcam");
                }

                configuredWebcams = configuredWebcamsLocal; // needed to preserve "final"
            }

        } catch (ParserConfigurationException | SAXException | XPathExpressionException |
                 IOException ex) {
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException " + eMessage);
        }
    }

    // The FTC SDK uses a string such as "Webcam 1" to connect
    // a webcam via the HardwareMap and to create a WebcamName
    // object. Use the webcam's serial number in its WebcamName
    // object to associate the webcam with its counterpart in
    // RobotConfig.xml.
    private void matchHardwareWebcamsWithConfiguredWebcams(EnumMap<RobotConstantsDecode.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> pConfiguredWebcams) {
        String webcamId;
        for (int i = 1; i <= pConfiguredWebcams.size(); i++) {
            webcamId = "Webcam " + new DecimalFormat("0").format(i);
            WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamId);
            if (!webcamName.isWebcam() || !webcamName.isAttached())
                throw new AutonomousRobotException(TAG, "Webcam " + webcamId +
                        " is not a webcam or is not attached");

            //RobotLogCommon.d(TAG, "Webcam hardware device " + webcamId +
            //        " is attached to webcam serial number " + webcamName.getSerialNumber());

            Optional<VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcam = pConfiguredWebcams.values().stream()
                    .filter(webcam -> webcam.serialNumber.equals(webcamName.getSerialNumber().toString()))
                    .findFirst();

            if (!configuredWebcam.isPresent())
                throw new AutonomousRobotException(TAG,
                        "No configured webcam for serial number: " + webcamName.getSerialNumber());

            // Now that we have the correct association, add the WebcamName to the
            // configured camera.
            VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcamObject = configuredWebcam.get();
            configuredWebcamObject.setWebcamName(webcamName);
            RobotLogCommon.i(TAG, "Webcam hardware device " + webcamId +
                    " is associated by serial number " + webcamName.getSerialNumber() +
                    " with configured webcam " + configuredWebcamObject.internalWebcamId);
        }
    }

    // Fields captured from RobotConfig.xml.
    public static class TeleOpSettings {
        public final RobotLogCommon.CommonLogLevel logLevel;
        public final double driveTrainPowerHigh, driveTrainPowerLow;

        public TeleOpSettings(String pLogLevel,
                              double pDriveTrainPowerHigh, double pDriveTrainPowerLow) {
            logLevel = RobotLogCommon.CommonLogLevel.valueOf(pLogLevel);
            driveTrainPowerHigh = pDriveTrainPowerHigh;
            driveTrainPowerLow = pDriveTrainPowerLow;
        }
    }

}

