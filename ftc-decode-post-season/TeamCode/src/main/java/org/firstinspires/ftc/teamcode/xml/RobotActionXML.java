package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

public class RobotActionXML {

    public static final String TAG = RobotActionXML.class.getSimpleName();

    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public RobotActionXML(String pRobotActionFilename) throws ParserConfigurationException, SAXException, IOException {

    /*
    // IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser (DTD or schema),
        // which the IntelliJ parser is.
        dbFactory.setIgnoringElementContentWhitespace(true);
    // End IntelliJ only
    */

        // Android or IntelliJ
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        //## ONLY works with a validating parser (DTD or schema),
        // which the Android Studio parser is not.
        // dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android or IntelliJ

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(pRobotActionFilename));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    // Find the requested opMode in the RobotAction.xml file.
    // Package and return all data associated with the OpMode.
    public RobotActionData getOpModeData(String pOpMode) throws XPathExpressionException {

        RobotLogCommon.CommonLogLevel logLevel = RobotLogCommon.CommonLogLevel.off; // default
        StartingPositionData startingPositionData = null;
        List<RobotXMLElement> actions = new ArrayList<>();

        // Use XPath to locate the desired OpMode.
        String opModePath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']";
        Node opModeNode = (Node) xpath.evaluate(opModePath, document, XPathConstants.NODE);
        if (opModeNode == null)
            throw new AutonomousRobotException(TAG, "Missing OpMode " + pOpMode);

        RobotLogCommon.c(TAG, "Extracting data from RobotAction.xml for OpMode " + pOpMode);

        // The next element in the XML is required: <parameters>
        Node parametersNode = XMLUtils.getNextElement(opModeNode.getFirstChild());
        if ((parametersNode == null) || !parametersNode.getNodeName().equals("parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <parameters> element");

        // The possible elements under <parameters> are:
        //   <log_level>
        //   <starting_position>
        // All are optional.

        // A missing or empty optional logging_level will return a log level
        // of "off".
        Node nextParameterNode = XMLUtils.getNextElement(parametersNode.getFirstChild());
        if ((nextParameterNode != null) && (nextParameterNode.getNodeName().equals("log_level"))) {
            String logLevelString = nextParameterNode.getTextContent().trim();
            logLevel = RobotLogCommon.CommonLogLevel.valueOf(logLevelString);
            nextParameterNode = XMLUtils.getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element is <starting_position>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("starting_position")) {
            // Get the value from each child of the starting_position element:
            // <x>79.0</x>
            // <y>188.0</y>
            // <angle>0.0</angle>
            double x;
            double y;
            double angle;
            Node xNode = XMLUtils.getNextElement(nextParameterNode.getFirstChild());
            if ((xNode == null) || !xNode.getNodeName().equals("x") || xNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'x' missing or empty");

            try {
                x = Double.parseDouble(xNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'x'");
            }

            Node yNode = XMLUtils.getNextElement(xNode.getNextSibling());
            if ((yNode == null) || !yNode.getNodeName().equals("y") || yNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'y' missing or empty");

            try {
                y = Double.parseDouble(yNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'y'");
            }

            Node angleNode = XMLUtils.getNextElement(yNode.getNextSibling());
            if ((angleNode == null) || !angleNode.getNodeName().equals("angle") || angleNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'angle' missing or empty");

            try {
                angle = Double.parseDouble(angleNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'angle'");
            }

            startingPositionData = new StartingPositionData(x, y, angle);
            nextParameterNode = XMLUtils.getNextElement(nextParameterNode.getNextSibling());
        }

        // Make sure there are no extraneous elements.
        if (nextParameterNode != null)
            throw new AutonomousRobotException(TAG, "Unrecognized element under <parameters>");

        // Now proceed to the <actions> element of the selected OpMode.
        String actionsPath = opModePath + "/actions";
        Node actionsNode = (Node) xpath.evaluate(actionsPath, document, XPathConstants.NODE);
        if (actionsNode == null)
            throw new AutonomousRobotException(TAG, "Missing <actions> element");

        // Now iterate through the children of the <actions> element of the selected OpMode.
        NodeList actionChildren = actionsNode.getChildNodes();
        Node actionNode;

        RobotXMLElement actionXMLElement;
        for (int i = 0; i < actionChildren.getLength(); i++) {
            actionNode = actionChildren.item(i);

            if (actionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) actionNode);
            actions.add(actionXMLElement);
        }

        return new RobotActionData(logLevel, startingPositionData, actions);
    }

    // Helper method to convert a nested <image_parameters> element into a class.
    public VisionParameters.ImageParameters
    getImageParametersFromXPath(RobotXMLElement pElement, String pPath) throws XPathExpressionException {
        Node ipNode = (Node) xpath.evaluate(pPath, pElement.getRobotXMLElement(), XPathConstants.NODE);
        if (ipNode == null)
            throw new AutonomousRobotException(TAG, "Missing " + pPath + " element");

        if (!ipNode.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Expected <image_parameters> element");

        return ImageXML.parseImageParameters(ipNode);
    }

    public static ArrayList<Pair<RobotConstantsDecode.ProcessorIdentifier, Boolean>> getStartWebcamProcessors(RobotXMLElement pStartWebcamElement) {

        // Get the <processor_set> child of the <START_WEBCAM>.
        NodeList processor_set_node = pStartWebcamElement.getRobotXMLElement().getElementsByTagName("processor_set");
        if (processor_set_node == null || processor_set_node.getLength() != 1)
            throw new AutonomousRobotException(TAG, "START_WEBCAM does not have exactly one processor_set child");

        // Process all of the <processor> children of the <processor_set> element.
        NodeList processor_nodes = processor_set_node.item(0).getChildNodes();
        if (processor_nodes == null)
            throw new AutonomousRobotException(TAG, "Element <processor_set> has no child elements");

        ArrayList<Pair<RobotConstantsDecode.ProcessorIdentifier, Boolean>> webcamProcessors = new ArrayList<>();

        XMLUtils.processElements(processor_nodes, (processor_node) -> {
            if (processor_node != null && processor_node.getNodeName().equals("processor")) {
                if (processor_node.getTextContent().isEmpty())
                    throw new AutonomousRobotException(TAG, "Element 'processor' is empty");

                // Look for the "disable_on_start" attribute; if it is present and
                // its value is "yes" then set the processor to be disabled when the
                // webcam starts.
                boolean enableOnStart = true; // default
                NamedNodeMap processor_attributes = processor_node.getAttributes();
                Node disable_node = processor_attributes.getNamedItem("disable_on_start");
                if (disable_node != null && disable_node.getTextContent().equals("yes"))
                    enableOnStart = false;

                RobotConstantsDecode.ProcessorIdentifier processorId =
                        RobotConstantsDecode.ProcessorIdentifier.valueOf(processor_node.getTextContent().toUpperCase());
                webcamProcessors.add(Pair.create(processorId, enableOnStart));
            }
        });

        // The collection of processors must not be empty.
        if (webcamProcessors.isEmpty())
            throw new AutonomousRobotException(TAG, "Element <processor_set> has no child <processor> elements");

        return webcamProcessors;
    }

    public static class RobotActionData {
        public final RobotLogCommon.CommonLogLevel logLevel;
        public final StartingPositionData startingPositionData;
        public final List<RobotXMLElement> actions;

        public RobotActionData(RobotLogCommon.CommonLogLevel pLogLevel,
                               StartingPositionData pStartingPositionData,
                               List<RobotXMLElement> pActions) {
            logLevel = pLogLevel;
            startingPositionData = pStartingPositionData;
            actions = pActions;
        }
    }

    public static class StartingPositionData {

        public final double x; // FTC field coordinates
        public final double y; // FTC field coordinates
        public final double angle; // with respect to the wall

        public StartingPositionData(double pStartingX, double pStartingY, double pStartingAngle) {
            x = pStartingX;
            y = pStartingY;
            angle = pStartingAngle;
        }
    }

}