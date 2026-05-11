package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButtonNWay;
import org.firstinspires.ftc.teamcode.xml.StartParameters;
import org.firstinspires.ftc.teamcode.xml.StartParametersXMLIn;
import org.firstinspires.ftc.teamcode.xml.StartParametersXMLOut;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumSet;

import javax.xml.parsers.ParserConfigurationException;

// OpMode to set the start parameters for the competition OpModes.
// This implies that the robot config file name is assumed to be
// "RobotConfig COMP.xml" and the robot action file name is
// "RobotAction COMP.xml".

// A class that supports start parameters for Decode Autonomous.
@TeleOp(name = "SetStartParameters", group = "Configure")
//@Disabled
public class SetStartParameters extends LinearOpMode {
    private static final String TAG = SetStartParameters.class.getSimpleName();

    private StartParameters startParameters;
    private boolean startParametersChanged = false;

    private FTCButton factoryResetButton;
    private FTCButton factoryResetConfirmButton;
    private FTCButton factoryResetCancelButton;
    private boolean factoryResetRequested = false;

    private static final int MAX_DELAY = 10; // seconds
    private int currentStartDelay;
    private FTCButton increaseDelayButton;
    private FTCButton decreaseDelayButton;

    private FTCToggleButtonNWay<StartParameters.AutoNamedSequence> autoSequenceToggleButton;
    private FTCToggleButtonNWay<StartParameters.AutoNamedPosition> autoNearStopPositionToggleButton;
    private FTCToggleButtonNWay<StartParameters.AutoNamedPosition> autoFarStartPositionToggleButton;

    @Override
    public void runOpMode() {
        RobotLogCommon.c(TAG, "Initializing SetStartParameters");

        factoryResetButton = new FTCButton(() -> gamepad1.start);
        factoryResetConfirmButton = new FTCButton(() -> gamepad1.y);
        factoryResetCancelButton = new FTCButton(() -> gamepad1.x);

        increaseDelayButton = new FTCButton(() -> gamepad1.dpad_up);
        decreaseDelayButton = new FTCButton(() -> gamepad1.dpad_down);

        autoSequenceToggleButton = new FTCToggleButtonNWay<>(() ->
                gamepad1.dpad_left, EnumSet.allOf(StartParameters.AutoNamedSequence.class));

        autoNearStopPositionToggleButton = new FTCToggleButtonNWay<>(() ->
                gamepad1.left_bumper, EnumSet.allOf(StartParameters.AutoNamedPosition.class));

        autoFarStartPositionToggleButton = new FTCToggleButtonNWay<>(() ->
                gamepad1.right_bumper, EnumSet.allOf(StartParameters.AutoNamedPosition.class));

        initializeStartParameters();

        while (!isStarted() && !isStopRequested()) {
            updateTelemetry();
            updateButtons();
            updateActions();
        }

        // The driver has hit the play button.
        if (opModeIsActive()) {
            if (startParametersChanged) {
                try {
                    new StartParametersXMLOut(startParameters,
                            WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR);
                } catch (ParserConfigurationException pe) {
                    throw new AutonomousRobotException(TAG, pe.getMessage());
                }

                RobotLogCommon.i(TAG, "Writing StartParameters.xml");
                telemetry.addLine("Writing StartParameters.xml");
            } else {
                RobotLogCommon.i(TAG, "No changes to StartParameters.xml");
                telemetry.addLine("No changes to StartParameters.xml");
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
        increaseDelayButton.update();
        decreaseDelayButton.update();
        autoSequenceToggleButton.update();
        autoNearStopPositionToggleButton.update();
        autoFarStartPositionToggleButton.update();
    }

    private void updateActions() {
        if (factoryResetRequested) {
            if (!updateFactoryResetConfirm())
                updateFactoryResetCancel();
            return;
        } else
            updateFactoryReset();

        updateIncreaseStartDelay();
        updateDecreaseStartDelay();
        updateAutoSequence();
        updateNearStopPosition();
        updateFarStartPosition();
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
        startParametersChanged = true;

        startParameters = new StartParameters(StartParameters.ROBOT_CONFIG_COMP_FILENAME, StartParameters.ROBOT_ACTION_COMP_FILENAME, 0,
                StartParameters.AutoNamedSequence.DEFAULT, StartParameters.AutoNamedPosition.NONE, StartParameters.AutoNamedPosition.NONE);

        // Reset the toggles
        autoSequenceToggleButton.setToggleState(startParameters.getAutoNamedSequence());
        autoNearStopPositionToggleButton.setToggleState(startParameters.getAutoNearStopPosition());
        autoFarStartPositionToggleButton.setToggleState(startParameters.getAutoFarStartPosition());

        RobotLogCommon.i(TAG, "Factory reset of start parameters");
        telemetry.addLine("Factory reset of start parameters");
        telemetry.update();
        sleep(1500);

        return true;
    }

    private void updateFactoryResetCancel() {
        if (factoryResetCancelButton.is(FTCButton.State.TAP))
            factoryResetRequested = false;
    }

    private void updateIncreaseStartDelay() {
        if (increaseDelayButton.is(FTCButton.State.TAP)) {
            if (currentStartDelay < MAX_DELAY) {
                ++currentStartDelay;
                startParameters.setAutoStartDelay(currentStartDelay);
                startParametersChanged = true;
            }
        }
    }

    private void updateDecreaseStartDelay() {
        if (decreaseDelayButton.is(FTCButton.State.TAP)) {
            if (currentStartDelay > 0) {
                --currentStartDelay;
                startParameters.setAutoStartDelay(currentStartDelay);
                startParametersChanged = true;
            }
        }
    }

    private void updateAutoSequence() {
        if (autoSequenceToggleButton.is(FTCButton.State.TAP)) {
            autoSequenceToggleButton.toggle();
            startParameters.setAutoNamedSequence(autoSequenceToggleButton.getToggleState());
            startParametersChanged = true;
        }
    }

    private void updateNearStopPosition() {
        if (autoNearStopPositionToggleButton.is(FTCButton.State.TAP)) {
            autoNearStopPositionToggleButton.toggle();
            startParameters.setAutoNearStopPosition(autoNearStopPositionToggleButton.getToggleState());
            startParametersChanged = true;
        }
    }

    private void updateFarStartPosition() {
        if (autoFarStartPositionToggleButton.is(FTCButton.State.TAP)) {
            autoFarStartPositionToggleButton.toggle();
            startParameters.setAutoFarStartPosition(autoFarStartPositionToggleButton.getToggleState());
            startParametersChanged = true;
        }
    }

    private void initializeStartParameters() {
        startParametersChanged = false;

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        StartParametersXMLIn startParametersXML;
        try {
            startParametersXML = new StartParametersXMLIn(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new AutonomousRobotException(TAG, e.getMessage());
        }

        startParameters = startParametersXML.getStartParameters();
        currentStartDelay = startParameters.getAutoStartDelay();
        autoSequenceToggleButton.setToggleState(startParameters.getAutoNamedSequence());
        autoNearStopPositionToggleButton.setToggleState(startParameters.getAutoNearStopPosition());
        autoFarStartPositionToggleButton.setToggleState(startParameters.getAutoFarStartPosition());
    }

    private void updateTelemetry() {

        // If a factory reset has been requested only show the confirm/cancel options.
        if (factoryResetRequested) {
            telemetry.addLine("Factory reset requested");
            telemetry.addLine("  Press Y to confirm, X to cancel");
            telemetry.update();
            return;
        }

        telemetry.addLine("The current start delay is " + startParameters.getAutoStartDelay());
        telemetry.addLine("Press DPAD_UP to increase delay; DPAD_DOWN to decrease");

        telemetry.addLine("The current Auto sequence is " + startParameters.getAutoNamedSequence());
        telemetry.addLine("Press DPAD_LEFT to change the autonomous sequence");

        telemetry.addLine("The current near stop position is " + startParameters.getAutoNearStopPosition());
        telemetry.addLine("Press LEFT BUMPER to change the stop position");

        telemetry.addLine("The current far start position is " + startParameters.getAutoFarStartPosition());
        telemetry.addLine("Press RIGHT BUMPER to change the start position");

        telemetry.addLine("Press START for factory reset");
        telemetry.addLine("Touch play to SAVE changes and END the OpMode");
        telemetry.update();
    }
}
