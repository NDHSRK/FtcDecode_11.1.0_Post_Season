package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

// Subset of the c++ ShapeDrawing class.
public class ShapeDrawing {
    private static final String TAG = ShapeDrawing.class.getSimpleName();

    public static void drawShapeContours(List<MatOfPoint> pContours, Mat pContoursOut) {
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pContoursOut, pContours, i, color, 2);
        }
    }

    public static void drawOneContour(List<MatOfPoint> pContours, int pContourIndex,
                                      Mat pContourOut) {
        Imgproc.drawContours(pContourOut, pContours, pContourIndex, new Scalar(0, 255, 0), 2);
    }

    public static void drawEnclosingCircles(List<MatOfPoint> pContours, Mat pCirclesOut) {
        RobotLogCommon.d(TAG, "drawEnclosingCircles: number of contours " + pContours.size());

        if (pContours.isEmpty())
            return;

        Point center = new Point();
        float[] radius = new float[1];

        Scalar color = new Scalar(0, 255, 0); // Green - good against dark background
        for (MatOfPoint oneContour : pContours) {
            // Convert MatOfPoint to MatOfPoint2f as required by minEnclosingCircle.
            MatOfPoint2f contour2f = new MatOfPoint2f(oneContour.toArray());

            Imgproc.minEnclosingCircle(contour2f, center, radius);
            Imgproc.circle(pCirclesOut, center, (int) radius[0], color, 2);
        }
    }

    public static void drawOneEnclosingCircle(List<MatOfPoint> pContours, int pContourIndex, Mat pCircleOut) {
        Point center = new Point();
        float[] radius = new float[1];

        // Convert MatOfPoint to MatOfPoint2f as required by minEnclosingCircle.
        MatOfPoint2f contour2f = new MatOfPoint2f(pContours.get(pContourIndex).toArray());

        Imgproc.minEnclosingCircle(contour2f, center, radius);
        Imgproc.circle(pCircleOut, center, (int) radius[0], new Scalar(0, 255, 0), 2);
    }

    // Draw rotated rectangles around contours, i.e. bounding rectangles with minimum area.
    public static void drawRotatedRectangles(List<MatOfPoint> pContours, Mat pRectanglesOut) {
        RobotLogCommon.d(TAG, "drawRotatedRectangles: number of contours " + pContours.size());

        if (pContours.isEmpty())
            return;

        // Get a RotatedRect for each contour and collect the RotatedRects for drawing.
        List<MatOfPoint> rrCollection = new ArrayList<>();
        for (MatOfPoint oneContour : pContours) {
            collectRotatedRectanglePoints(oneContour, rrCollection);
        }

        // Use drawContours to draw the RotatedRects.
        Imgproc.drawContours(pRectanglesOut, rrCollection, -1, new Scalar(0, 255, 0), 2);
    }

    public static void drawOneRotatedRectangle(MatOfPoint pContour, Mat pRectangleOut) {
        List<MatOfPoint> rrCollection = new ArrayList<>();
        collectRotatedRectanglePoints(pContour, rrCollection);

        // Use drawContours to draw the RotatedRect.
        Imgproc.drawContours(pRectangleOut, rrCollection, 0, new Scalar(0, 255, 0), 2);
    }

    public static void drawOneRotatedRectangle(RotatedRect pRR, Mat pRectangleOut) {
        List<MatOfPoint> rrCollection = new ArrayList<>();

        // Add points for one RotatedRect to the collection.
        MatOfPoint rrPoints = getRotatedRectanglePoints(pRR);
        rrCollection.add(rrPoints);

        // Use drawContours to draw the RotatedRects.
        Imgproc.drawContours(pRectangleOut, rrCollection, 0, new Scalar(0, 255, 0), 2);
    }

    public static void drawOneCircle(Point pCenter, int pRadius, Mat pCircleOut) {
        RobotLogCommon.d(TAG, "Draw one circle at " + pCenter.x + ", y " + pCenter.y);
        Imgproc.circle(pCircleOut, pCenter, pRadius, new Scalar(0, 255, 0), 2);
    }

    // Use this to show errors such as no contours or violations of criteria
    // such as size.
    public static void drawX(Mat pImageOut, Scalar pBGRColor) {

        double DRAWN_X_SIZE_FACTOR = 0.33;
        int drawnXThickness = 2;
        int roiWidth = pImageOut.cols();
        int roiHeight = pImageOut.rows();

        // Set the Points for the X.
        Point xUpperLeft = new Point(roiWidth * DRAWN_X_SIZE_FACTOR, roiHeight * DRAWN_X_SIZE_FACTOR);
        Point xUpperRight = new Point(roiWidth * (1.0 - DRAWN_X_SIZE_FACTOR), xUpperLeft.y);
        Point xLowerLeft = new Point(xUpperLeft.x, roiHeight * (1.0 - DRAWN_X_SIZE_FACTOR));
        Point xLowerRight = new Point(xUpperRight.x, xLowerLeft.y);

        Imgproc.line(pImageOut, xUpperLeft, xLowerRight, pBGRColor, drawnXThickness);
        Imgproc.line(pImageOut, xLowerLeft, xUpperRight, pBGRColor, drawnXThickness);
    }

    private static void collectRotatedRectanglePoints(MatOfPoint pContour, List<MatOfPoint> pRRCollection) {
        MatOfPoint2f contourPoints2f = new MatOfPoint2f(pContour.toArray()); // minAreaRect requires MatOfPoint2f
        RotatedRect oneRotatedRect = Imgproc.minAreaRect(contourPoints2f);

        // Convert the array of points into a MatOfPoint acceptable to drawContours.
        MatOfPoint rrPoints = getRotatedRectanglePoints(oneRotatedRect);
        pRRCollection.add(rrPoints);
    }

    private static MatOfPoint getRotatedRectanglePoints(RotatedRect pRR) {
        Point[] rectPoints = new Point[4];
        pRR.points(rectPoints);

        RobotLogCommon.d(TAG, "Enclosing contour in RotatedRect with center " +
                pRR.center);
        RobotLogCommon.d(TAG, "Rectangle area " + (pRR.size.width * pRR.size.height));
        RobotLogCommon.d(TAG, "Rotated rect points 0 " + rectPoints[0] +
                ", 1 " + rectPoints[1] + ", 2 " + rectPoints[2] + ", 3 " + rectPoints[3]);

        // Convert the array of points into a MatOfPoint acceptable to drawContours.
        return new MatOfPoint(rectPoints);
    }

// Functions from CLOpenCVTestbed4 not implemented here.
/*
void ShapeDrawing::drawCorners(const vector<Point2f>& pCorners, Mat& pCornersOut)
void ShapeDrawing::drawCircles(const vector<Vec3f>& pCircles, Mat& pCirclesOut);
void ShapeDrawing::drawOneRectangle(const Rect& pRectangle, Mat& pRectangleOut);
void ShapeDrawing::drawOneLine(const Vec4i& pOneLine, Mat& pLineOut); {
*/


}
