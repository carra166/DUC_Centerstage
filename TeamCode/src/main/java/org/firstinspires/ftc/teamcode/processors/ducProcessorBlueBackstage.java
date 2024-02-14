package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class ducProcessorBlueBackstage implements VisionProcessor {

    public Scalar redLower = new Scalar(82.2, 172.8, 131.8);
    public Scalar redUpper = new Scalar(134.6, 255.0, 255.0);

    public Rect theFirstOne = new Rect(10, 370, 40, 40);
    public Rect theSecondOne = new Rect(330, 375, 40, 40);
    public Rect theThirdOne = new Rect(600, 200, 40, 40);

    public Mat redFirst = new Mat();
    public Mat redSecond = new Mat();
    public Mat redThird = new Mat();

    boolean tuning = false;
    public Mat hsv = new Mat();
    public Mat threshold = new Mat();

    public ArrayList<MatOfPoint> contours = new ArrayList<>();

    public double duckPosition = 1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (tuning) {
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
            Core.inRange(frame, redLower, redUpper, frame);
            redFirst = new Mat(frame, theFirstOne);
            redSecond = new Mat(frame, theSecondOne);
            redThird = new Mat(frame, theThirdOne);
        } else {
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, redLower, redUpper, threshold);
            redFirst = new Mat(threshold, theFirstOne);
            redSecond = new Mat(threshold, theSecondOne);
            redThird = new Mat(threshold, theThirdOne);
        }
        Imgproc.rectangle(frame, theFirstOne, new Scalar(100,0,222));
        Imgproc.rectangle(frame, theSecondOne, new Scalar(100,0,222));
        Imgproc.rectangle(frame, theThirdOne, new Scalar(100,0,222));

            duckPosition = 1;

            //AREA 1
            detectContours(redFirst, theFirstOne, contours, frame, 1);

            //AREA 2
            detectContours(redSecond, theSecondOne, contours, frame, 2);

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    private void detectContours(Mat box, Rect rectangle, ArrayList<MatOfPoint> contours, Mat frame, double number) {

        Imgproc.findContours(box, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    frame,
                    new Point(cX - (width/2) + rectangle.x, cY - (height/2) + rectangle.y),
                    new Point(cX + (width/2) + rectangle.x, cY + (height/2) + rectangle.y),
                    new Scalar(240,240,240),
                    2
            );
        }

        Imgproc.putText(frame, Integer.toString(contours.size()), new Point(rectangle.x, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        if (contours.size() > 0) {
            duckPosition = number + 1;
        }
        contours.clear();

    }

    public double getDuckPosition() {
        return duckPosition;
    }
}