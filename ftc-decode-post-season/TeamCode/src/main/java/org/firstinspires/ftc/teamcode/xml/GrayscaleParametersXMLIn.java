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

public class GrayscaleParametersXMLIn {

    public static final String TAG = GrayscaleParametersXMLIn.class.getSimpleName();
    private static final String FILE_NAME = "GrayscaleParameters.xml";

    // Common
    private final VisionParameters.ImageParameters imageParameters;
    private final VisionParameters.GrayParameters grayscaleParameters;

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    public GrayscaleParametersXMLIn(String pXMLDirectory) throws ParserConfigurationException, SAXException, IOException {

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
        Element grayscaleParametersRoot = document.getDocumentElement();
        if (grayscaleParametersRoot == null || !grayscaleParametersRoot.getNodeName().equals("grayscale_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <grayscale_parameters> element");

        // Parse <image_parameters>
        Node ip_node = XMLUtils.getNextElement(grayscaleParametersRoot.getFirstChild());
        if ((ip_node == null) || !ip_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <image_parameters> element");

        imageParameters = ImageXML.parseImageParameters(ip_node);

        // Parse <gray_parameters>
        Node gp_node = XMLUtils.getNextElement(ip_node.getNextSibling());
        if ((gp_node == null) || !gp_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <gray_parameters> element");

        grayscaleParameters = ImageXML.parseGrayParameters(gp_node);

        RobotLogCommon.c(TAG, "In GrayscaleParametersXMLIn; read in  + FILE_NAME");
    }

    public VisionParameters.ImageParameters getImageParameters() {
        return imageParameters;
    }

    public VisionParameters.GrayParameters getGrayscaleParameters() {
        return grayscaleParameters;
    }

}
