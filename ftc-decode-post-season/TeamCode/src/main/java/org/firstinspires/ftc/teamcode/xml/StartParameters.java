package org.firstinspires.ftc.teamcode.xml;

// Starting parameters that can be set via a dedicated TeleOp
// OpMode and read by either an Autonomous or TeleOp competition
// OpMode.
public class StartParameters {

    public static final String TAG = StartParameters.class.getSimpleName();

    public static final String ROBOT_CONFIG_COMP_FILENAME = "RobotConfig COMP.xml";
    public static final String ROBOT_ACTION_COMP_FILENAME = "RobotAction COMP.xml";

    public enum AutoNamedSequence {DEFAULT, WATERMELON, QUALIA, CYBERKNIGHTS, BEYOND}

    // Named positions are used for the position of
    // the last three artifacts to pick up when the
    // robot starts at the near goal and the position
    // of the first three artifacts to pick up when
    // the robot starts at the far goal.
    // Row designations are relative to the goals.
    public enum AutoNamedPosition {
        NONE, ROW_1, ROW_2, ROW_3, CORNER
    }

    public final String robotConfigFilename;
    public final String robotActionFilename;
    private int autoStartDelay;

    private AutoNamedSequence autoNamedSequence;
    private AutoNamedPosition autoNearStopPosition;
    private AutoNamedPosition autoFarStartPosition;

    public StartParameters(String pRobotConfigFilename, String pRobotActionFilename, int pAutoStartDelay,
                           AutoNamedSequence pAutoNamedSequence,
                           AutoNamedPosition pAutoNearStopPosition, AutoNamedPosition pAutoFarStartPosition) {
        robotConfigFilename = pRobotConfigFilename;
        robotActionFilename = pRobotActionFilename;
        autoStartDelay = pAutoStartDelay;
        autoNamedSequence = pAutoNamedSequence;
        autoNearStopPosition = pAutoNearStopPosition;
        autoFarStartPosition = pAutoFarStartPosition;
    }

    public int getAutoStartDelay() {
        return autoStartDelay;
    }

    public void setAutoStartDelay(int pAutoStartDelay) {
        autoStartDelay = pAutoStartDelay;
    }

    public AutoNamedSequence getAutoNamedSequence() {
        return autoNamedSequence;
    }

    public void setAutoNamedSequence(AutoNamedSequence pAutoNamedSequence) {
        autoNamedSequence = pAutoNamedSequence;
    }

    public AutoNamedPosition getAutoNearStopPosition() {
        return autoNearStopPosition;
    }

    public void setAutoNearStopPosition(AutoNamedPosition pAutoNearStopPosition) {
        autoNearStopPosition = pAutoNearStopPosition;
    }

    public AutoNamedPosition getAutoFarStartPosition() {
        return autoFarStartPosition;
    }

    public void setAutoFarStartPosition(AutoNamedPosition pAutoFarStartPosition) {
        autoFarStartPosition = pAutoFarStartPosition;
    }


}