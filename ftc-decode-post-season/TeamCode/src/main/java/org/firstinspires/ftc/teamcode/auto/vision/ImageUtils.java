package org.firstinspires.ftc.teamcode.auto.vision;

import android.graphics.Bitmap;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.xml.VisionParameters;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;
import java.util.stream.Collectors;

public class ImageUtils {

    public static final String TAG = ImageUtils.class.getSimpleName();

    public static Bitmap getBitmapFromMat(Mat pVideoFrameMat, int pVideoFrameHeight, int pVideoFrameWidth) {
        Bitmap outputBitmap = Bitmap.createBitmap(pVideoFrameWidth, pVideoFrameHeight, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(pVideoFrameMat, outputBitmap);
        return outputBitmap;
    }

    // Load an image from a file. Only used for test cases since we
    // normally get images from a camera.
    public static Mat loadImage(String pInputFilename, int pImageFormat) {
        RobotLogCommon.i(TAG, "File name " + pInputFilename);

        // Sanity check on the file name extension.
        if (!(pInputFilename.endsWith(".png") || pInputFilename.endsWith(".jpg")))
            throw new AutonomousRobotException(TAG, "Invalid image file name");

        Mat imageOut = Imgcodecs.imread(pInputFilename, pImageFormat);
        if (imageOut.empty())
            throw new AutonomousRobotException(TAG, "Could not find or open the image " + pInputFilename);

        RobotLogCommon.d(TAG, "Image width " + imageOut.cols() + ", height " + imageOut.rows());
        return imageOut;
    }

    public static String createOutputFilePreamble(String pWorkingDirectory, String pInputFilename, String pFileDate) {
        // Sanity check on the file name extension.
        if (!(pInputFilename.endsWith(".png") || pInputFilename.endsWith(".jpg")))
            throw new AutonomousRobotException(TAG, "Invalid image file name");

        // Strip the extension from the end of the file name and append an underscore and time stamp.
        String originalFN = pInputFilename.substring(0, pInputFilename.lastIndexOf('.'));
        return pWorkingDirectory + originalFN + "_" + pFileDate;
    }

    public static Mat preProcessImage(Mat pOriginalImage, VisionParameters.ImageParameters pImageParameters) {
        if ((pOriginalImage.cols() != pImageParameters.resolution_width) ||
                (pOriginalImage.rows() != pImageParameters.resolution_height))
            throw new AutonomousRobotException(TAG,
                    "Mismatch between actual image width and expected image width " + pImageParameters.resolution_width +
                            ", height " + pImageParameters.resolution_height);

        if (pImageParameters.image_roi.width == 0 || pImageParameters.image_roi.height == 0)
            throw new AutonomousRobotException(TAG, "At least one ROI dimension was 0");

        // If the ROI is the same size as the full image just return the full image.
        if (pImageParameters.image_roi.width == pImageParameters.resolution_width &&
                pImageParameters.image_roi.height == pImageParameters.resolution_height)
            return pOriginalImage;

        // Crop the image to reduce distractions.
        Rect roiDefinition =
                new Rect(pImageParameters.image_roi.x,
                        pImageParameters.image_roi.y,
                        pImageParameters.image_roi.width,
                        pImageParameters.image_roi.height);

        Mat roi = new Mat(pOriginalImage, roiDefinition);
        RobotLogCommon.d(TAG, "Image ROI x " + roiDefinition.x + ", y " + roiDefinition.y + ", width " + roiDefinition.width + ", height " + roiDefinition.height);
        return roi;
    }

    //## Imported from IJCenterStageVision.
    //## This sharpening filter makes a difference in marginal cases.
    // From OpencvTestbed3 (cpp) GrayscaleTechnique
    // From https://stackoverflow.com/questions/27393401/opencv-in-java-for-image-filtering
    public static Mat sharpen(Mat pDullMat) {
        int kernelSize = 3;
        Mat kernel = new Mat(kernelSize, kernelSize, CvType.CV_32F) {
            {
                put(0, 0, 0);
                put(0, 1, -1);
                put(0, 2, 0);

                put(1, 0, -1);
                put(1, 1, 5);
                put(1, 2, -1);

                put(2, 0, 0);
                put(2, 1, -1);
                put(2, 2, 0);
            }
        };

        Mat sharpMat = new Mat();
        Imgproc.filter2D(pDullMat, sharpMat, -1, kernel);

        return sharpMat;
    }

    // Use inRange to threshold an HSV image to binary.
    // Account for hue ranges that cross the 180 degree boundary.
    // Red, for example, might have a hueLow of 170 and a hueHigh of 10.
    // See https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
    public static Mat applyInRange(Mat pHSVROI, VisionParameters.HSVParameters pHSVParameters) {
        int hueLow = pHSVParameters.hue_low;
        int hueHigh = pHSVParameters.hue_high;
        int satHigh = 255;
        int valHigh = 255;

        // Use inRange to threshold to binary.
        // Account for hue ranges that cross the 180 degree boundary.
        // Red, for example, might have a hueLow of 170 and a hueHigh of 10.
        // See https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
        Mat thresholded = new Mat();
        int inRangeSatLow = pHSVParameters.saturation_threshold_low;
        int inRangeValLow = pHSVParameters.value_threshold_low;
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: hue low " + hueLow + ", hue high " + hueHigh);
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: saturation low " + inRangeSatLow + ", value low " + inRangeValLow);

        // Sanity check for hue.
        if (!((hueLow >= 0 && hueLow <= 180) && (hueHigh >= 0 && hueHigh <= 180) &&
                (hueLow != hueHigh)))
            throw new AutonomousRobotException(TAG, "Hue out of range");

        // Normal hue range.
        if (hueLow < hueHigh)
            Core.inRange(pHSVROI, new Scalar(hueLow, inRangeSatLow, inRangeValLow), new Scalar(hueHigh, satHigh, valHigh), thresholded);
        else {
            // For a hue range from the XML file of low 170, high 10
            // the following yields two new ranges: 170 - 180 and 0 - 10.
            Mat range1 = new Mat();
            Mat range2 = new Mat();
            Core.inRange(pHSVROI, new Scalar(hueLow, inRangeSatLow, inRangeValLow), new Scalar(180, satHigh, valHigh), range1);
            Core.inRange(pHSVROI, new Scalar(0, inRangeSatLow, inRangeValLow), new Scalar(hueHigh, satHigh, valHigh), range2);
            Core.bitwise_or(range1, range2, thresholded);
        }

        return thresholded;
    }

    // 6/17/2025 Change of direction. Removed morphological opening and
    // blurring from this function. Depending on the application, these
    // can be applied before this function is called.
    // Note that pyimagesearch performs a 7x7 Gaussian blur before thresholding:
    // https://www.pyimagesearch.com/2021/04/28/opencv-thresholding-cv2-threshold/
    // This OpenCV tutorial does not:
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // And neither does this one:
    // https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
    public static Mat applyGrayThreshold(Mat pGrayInputROI, int pLowThreshold) {
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(pGrayInputROI, thresholded,
                Math.abs(pLowThreshold),
                255,   // white
                pLowThreshold >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type);
        return thresholded;
    }

    // Sort contours by area in descending order.
    public static List<MatOfPoint> sortContoursByArea(List<MatOfPoint> pContours) {
        List<MatOfPoint> sortedContours = pContours.stream()
                .sorted(Comparator.comparing(Imgproc::contourArea))
                .collect(Collectors.toList());

        // .sorted(Comparator.reverseOrder(Imgproc::contourArea)) is ambiguous
        Collections.reverse(sortedContours);
        return sortedContours;
    }

    // From https://stackoverflow.com/questions/40669684/opencv-sorting-contours-by-area-in-java
    // See lambda in https://stackoverflow.com/questions/24378646/finding-max-with-lambda-expression-in-java
    public static Optional<MatOfPoint> getLargestContour(List<MatOfPoint> pContours) {
        return pContours.stream().max(Comparator.comparing(Imgproc::contourArea));
    }

    public static Point getContourCentroid(MatOfPoint pOneContour) {
        Moments moments = Imgproc.moments(pOneContour);
        return new Point(moments.get_m10() / moments.get_m00(),
                moments.get_m01() / moments.get_m00());
    }

    // Implemented in CLOpenCVTestbed4; port if needed.
    /*
    static void applyHoughCircles(const Mat& pImageIn, vector<Vec3f>& pCirclesOut, ... );
    static void filterCircles(vector<Vec3f>& pCircles, double pMinRadius, double pMaxRadius);
    static vector<Vec4i> getLines(const Mat& pImageIn);
    static void getCorners(const Mat& pImageIn, vector<Point2f> pCorners);
    */

    // Reduce the number of contours in a thresholded image by filtering
    // out those that have an area less than a minimum.
    public static FilteredContoursRecord filterContours(Mat pThresholded, int pImageHeight, int pImageWidth, double pMinArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(pThresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        RobotLogCommon.d(TAG, "Number of contours before filtering " + contours.size());

        // Draw on an all-black background; drawContours requires a BGR image.
        Mat filteredBGR = Mat.zeros(pImageHeight, pImageWidth, CvType.CV_8UC3);

        int numFiltered = 0;
        Scalar allWhite = new Scalar(255, 255, 255);
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > pMinArea) {
                numFiltered++;
                Imgproc.drawContours(filteredBGR, contours, i, allWhite, Imgproc.FILLED);
            }
        }

        RobotLogCommon.d(TAG, "Number of contours after filtering " + numFiltered);

        // Convert the BGR image to grayscale, which in our case should be binary.
        Mat filteredBinary = new Mat();
        Imgproc.cvtColor(filteredBGR, filteredBinary, Imgproc.COLOR_BGR2GRAY);

        return new FilteredContoursRecord(contours.size(), numFiltered, filteredBinary);
    }

    public static class FilteredContoursRecord {
        public final int numUnfilteredContours;
        public final int numFilteredContours;
        public Mat filteredBinaryOutput;

        public FilteredContoursRecord(int pNumUnfilteredContours, int pNumFilteredContours,
                                      Mat pFilteredBinaryOutput) {
            numUnfilteredContours = pNumUnfilteredContours;
            numFilteredContours = pNumFilteredContours;
            filteredBinaryOutput = pFilteredBinaryOutput;
        }
    }

}

