package org.firstinspires.ftc.teamcode.common;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RobotConstantsDecode {

    public enum OpMode {
        // Decode Autonomous OpModes
        BLUE_NEAR(OpModeType.COMPETITION),
        BLUE_FAR(OpModeType.COMPETITION),
        RED_NEAR(OpModeType.COMPETITION),
        RED_FAR(OpModeType.COMPETITION),

        TEST(OpModeType.AUTO_TEST), TEST_PRE_MATCH(OpModeType.AUTO_TEST),
        AUTO_NO_DRIVE(OpModeType.AUTO_TEST),

        // TeleOp OpModes
        TELEOP_FULL(OpModeType.COMPETITION),
        TELEOP_NO_DRIVE(OpModeType.TELEOP_TEST),
        TELEOP_TEST(OpModeType.TELEOP_TEST),

        // Indication that an OpMode has not yet been assigned.
        OPMODE_NPOS(OpModeType.PSEUDO_OPMODE);

        public enum OpModeType {COMPETITION, AUTO_TEST, TELEOP_TEST, PSEUDO_OPMODE}

        private final OpModeType opModeType;

        OpMode(OpModeType pOpModeType) {
            opModeType = pOpModeType;
        }

        public OpModeType getOpModeType() {
            return opModeType;
        }
    }

    // The InternalCameraId identifies each unique camera and its position on
    // the robot.
    public enum InternalWebcamId {
        FRONT_WEBCAM, REAR_WEBCAM, WEBCAM_NPOS
    }

    public enum ProcessorIdentifier {
        RAW_FRAME, APRIL_TAG, CAMERA_STREAM_PREVIEW, PROCESSOR_NPOS
    }

    public static final int MAX_ARTIFACTS_IN_REVOLVER = 3;
}