package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {

    public static final String IMAGE_DIR = "/images/";
    public static final String LOG_DIR = "/logs/";
    public static final String XML_DIR = "/xml/";

    // For standard indented formatting of an XML file.
    public static final String XSLT_FILE_NAME = "StandardTransform.xslt";

    public static final int VV_LOGGING_SAMPLING_FREQUENCY = 5;

    public enum RunType {
        AUTONOMOUS, TELEOP, TELEOP_VISION_PREVIEW
    }

    public enum Alliance {
        NONE, BLUE, RED
    }

    public enum RecognitionResults {
        RECOGNITION_INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

}
