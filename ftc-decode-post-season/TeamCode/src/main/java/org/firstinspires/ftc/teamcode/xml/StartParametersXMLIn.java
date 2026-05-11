package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class StartParametersXMLIn {

    public static final String TAG = StartParametersXMLIn.class.getSimpleName();
    private static final String FILE_NAME = "StartParameters.xml";

    // Common
    private final StartParameters startParameters;

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    public StartParametersXMLIn(String pXMLDirectory) throws ParserConfigurationException, SAXException, IOException {

        // IntelliJ only
        /*
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
         */
        // End IntelliJ only

        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        String xmlFilePath = pXMLDirectory + FILE_NAME;

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        Document document = dBuilder.parse(new File(xmlFilePath));
        Element startParametersRoot = document.getDocumentElement();

        Node config_node = startParametersRoot.getFirstChild();
        config_node = XMLUtils.getNextElement(config_node);
        if (config_node == null || !config_node.getNodeName().equals("robot_config") || config_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_config' not found");

        String robotConfigFilename = config_node.getTextContent();

        Node action_node = config_node.getNextSibling();
        action_node = XMLUtils.getNextElement(action_node);
        if (action_node == null || !action_node.getNodeName().equals("robot_action") || action_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_action' not found");

        String robotActionFilename = action_node.getTextContent();

        Node local_delay_node = action_node.getNextSibling();
        local_delay_node = XMLUtils.getNextElement(local_delay_node);
        if (local_delay_node == null || !local_delay_node.getNodeName().equals("auto_start_delay") || local_delay_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'auto_start_delay' not found");

        String delayString = local_delay_node.getTextContent();
        int autoStartDelay;
        try {
            autoStartDelay = Integer.parseInt(delayString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'auto_start_delay'");
        }

        // Parse the <auto_sequence> element.
        Node sequence_node = local_delay_node.getNextSibling();
        sequence_node = XMLUtils.getNextElement(sequence_node);
        if (sequence_node == null || !sequence_node.getNodeName().equals("auto_named_sequence"))
            throw new AutonomousRobotException(TAG, "Element 'auto_named_sequence' not found");

        String sequenceText = sequence_node.getTextContent().trim().toUpperCase();
        StartParameters.AutoNamedSequence sequenceValue =
                StartParameters.AutoNamedSequence.valueOf(sequenceText);

        // Parse the <auto_near_stop_position> element.
        Node near_stop_node = sequence_node.getNextSibling();
        near_stop_node = XMLUtils.getNextElement(near_stop_node);
        if (near_stop_node == null || !near_stop_node.getNodeName().equals("auto_near_stop_position"))
            throw new AutonomousRobotException(TAG, "Element 'auto_near_stop_position' not found");

        String nearText = near_stop_node.getTextContent().trim().toUpperCase();
        StartParameters.AutoNamedPosition nearValue =
                StartParameters.AutoNamedPosition.valueOf(nearText);

        // Parse the <auto_far_start_position> element.
        Node far_start_node = near_stop_node.getNextSibling();
        far_start_node = XMLUtils.getNextElement(far_start_node);
        if (far_start_node == null || !far_start_node.getNodeName().equals("auto_far_start_position"))
            throw new AutonomousRobotException(TAG, "Element 'auto_far_start_position' not found");

        String farText = far_start_node.getTextContent().trim().toUpperCase();
        StartParameters.AutoNamedPosition farValue =
                StartParameters.AutoNamedPosition.valueOf(farText);

        startParameters = new StartParameters(robotConfigFilename, robotActionFilename, autoStartDelay,
                sequenceValue, nearValue, farValue);

        RobotLogCommon.c(TAG, "In StartParametersXML; opened and parsed the XML file");
    }

    public StartParameters getStartParameters() {
        return startParameters;
    }

}
