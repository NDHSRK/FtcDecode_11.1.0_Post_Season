package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class GrayscaleParametersXMLOut {

    public static final String TAG = GrayscaleParametersXMLOut.class.getSimpleName();
    private final String FILE_NAME = "GrayscaleParameters.xml";

    // The purpose of this class is to write out the file GrayscaleParameters.xml
    // from data structures created by the OpMode SetGrayscaleThreshold.
    public GrayscaleParametersXMLOut(VisionParameters.ImageParameters pImageParameters,
                                     VisionParameters.GrayParameters pGrayscaleParameters,
                                     String pXMLDirectory) throws ParserConfigurationException {
        // Android only
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        Document document = factory.newDocumentBuilder().newDocument();

        String xmlFilePath = pXMLDirectory + FILE_NAME;

        // Create elements and append them.
        Element root = document.createElement("grayscale_parameters");
        document.appendChild(root);

        // Create the <image_parameters> element
        Element ip = document.createElement("image_parameters");
        root.appendChild(ip);

        // <image_source>
        Element is = document.createElement("image_source");
        is.appendChild(document.createTextNode(pImageParameters.image_source));
        ip.appendChild(is);

        // <resolution>
        //   <width>
        //   <height
        Element re = document.createElement("resolution");
        ip.appendChild(re);

        Element we = document.createElement("width");
        we.appendChild(document.createTextNode(Integer.toString(pImageParameters.resolution_width)));
        re.appendChild(we);

        Element he = document.createElement("height");
        he.appendChild(document.createTextNode(Integer.toString(pImageParameters.resolution_height)));
        re.appendChild(he);

        /*
         <image_roi>
            <x>0</x>
            <y>0</y>
            <width>640</width>
            <height>480</height>
         </image_roi>
        */
        Element roi = document.createElement("image_roi");
        ip.appendChild(roi);

        // <x>
        Element roi_x = document.createElement("x");
        roi_x.appendChild(document.createTextNode(Integer.toString(pImageParameters.image_roi.x)));
        roi.appendChild(roi_x);

        // <y>
        Element roi_y = document.createElement("y");
        roi_y.appendChild(document.createTextNode(Integer.toString(pImageParameters.image_roi.y)));
        roi.appendChild(roi_y);

        // <width>
        Element roi_w = document.createElement("width");
        roi_w.appendChild(document.createTextNode(Integer.toString(pImageParameters.image_roi.width)));
        roi.appendChild(roi_w);

        // <height>
        Element roi_h = document.createElement("height");
        roi_h.appendChild(document.createTextNode(Integer.toString(pImageParameters.image_roi.height)));
        roi.appendChild(roi_h);

        // Create the <gray_parameters> element
        Element gp = document.createElement("gray_parameters");
        root.appendChild(gp);

        // Create <threshold_low>
        Element tl = document.createElement("threshold_low");
        tl.appendChild(document.createTextNode(Integer.toString(pGrayscaleParameters.threshold_low)));
        gp.appendChild(tl);

        XMLUtils.writeXMLFile(document, xmlFilePath, pXMLDirectory + RobotConstants.XSLT_FILE_NAME);

        RobotLogCommon.c(TAG, "In GrayscaleParametersXMLOut; created and wrote out  + FILE_NAME");
    }

}
