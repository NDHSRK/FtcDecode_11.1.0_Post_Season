package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class StartParametersXMLOut {

    public static final String TAG = StartParametersXMLOut.class.getSimpleName();
    private final String FILE_NAME = "StartParameters.xml";

    // The purpose of this class is to write out the file StartParameters3.xml
    // from a data structure instantiated by the OpMode SetStartParameters.
    public StartParametersXMLOut(StartParameters pStartParameters, String pXMLDirectory) throws ParserConfigurationException {
        // Android only
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        Document document = factory.newDocumentBuilder().newDocument();

        String xmlFilePath = pXMLDirectory + FILE_NAME;

        // Create elements and append them.
        Element root = document.createElement("StartParameters");
        document.appendChild(root);

        Element robotConfig = document.createElement("robot_config");
        root.appendChild(robotConfig);
        robotConfig.appendChild(document.createTextNode(pStartParameters.robotConfigFilename));

        Element robotAction = document.createElement("robot_action");
        root.appendChild(robotAction);
        robotAction.appendChild(document.createTextNode(pStartParameters.robotActionFilename));

        Element autoStartDelay = document.createElement("auto_start_delay");
        root.appendChild(autoStartDelay);
        autoStartDelay.appendChild(document.createTextNode(Integer.toString(pStartParameters.getAutoStartDelay())));

        Element autoNamedSequence = document.createElement("auto_named_sequence");
        root.appendChild(autoNamedSequence);
        autoNamedSequence.appendChild(document.createTextNode(pStartParameters.getAutoNamedSequence().toString()));

        Element autoNearStopPosition = document.createElement("auto_near_stop_position");
        autoNearStopPosition.appendChild(document.createTextNode(pStartParameters.getAutoNearStopPosition().toString()));
        root.appendChild(autoNearStopPosition);

        Element autoFarStartPosition = document.createElement("auto_far_start_position");
        autoFarStartPosition.appendChild(document.createTextNode(pStartParameters.getAutoFarStartPosition().toString()));
        root.appendChild(autoFarStartPosition);

        XMLUtils.writeXMLFile(document, xmlFilePath, pXMLDirectory + RobotConstants.XSLT_FILE_NAME);

        RobotLogCommon.c(TAG, "In StartParametersXMLOut; created and wrote out  + FILE_NAME");
    }

}
