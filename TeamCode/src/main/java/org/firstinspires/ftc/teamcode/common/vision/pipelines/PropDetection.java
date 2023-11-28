package org.firstinspires.ftc.teamcode.common.vision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.vision.util.PropPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PropDetection extends OpenCvPipeline {

    boolean blueOrRed;
    Telemetry telemetry;
    PropPosition position = PropPosition.NONE;

    public PropDetection(Telemetry telemetry, boolean blue) {
        this.telemetry = telemetry;
        this.blueOrRed = blue;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        //blur first
        Imgproc.GaussianBlur(input, input, new Size(15, 15), 0, 0);

        //threshold for prop color
        Mat threshold = thresholdForColor(hsvMat);

        //dilate for visibility
        Mat dilatedOutput = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(17, 17));
        Imgproc.dilate(threshold, dilatedOutput, kernel, new Point(-1, -1), 1); //Image dilation too much

        //find & draw contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(dilatedOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 3);

        //figure out where the prop is based on contours
        findProp(contours, input);

        //mask the input image based on our threshold
        Mat output = new Mat();
        Core.bitwise_and(input, input, output, dilatedOutput);

        //save mask for debugging
        Imgcodecs.imwrite("sdcard/FIRST/mask.png", threshold);

        //help RAM out lol
        releaseMats(hsvMat, threshold, dilatedOutput, kernel, hierarchy, output);

        return input;
    }

    public Mat thresholdForColor(Mat hsvMat) {
        Mat threshold = new Mat();

        if (blueOrRed) {
            Scalar lower_blue = new Scalar(85, 50, 40);
            Scalar upper_blue = new Scalar(135, 255, 255);

            Core.inRange(hsvMat, lower_blue, upper_blue, threshold);

            return threshold;
        }
        Scalar lower_red = new Scalar(0, 70, 50);
        Scalar upper_red = new Scalar(10, 255, 255);

        Scalar lower_red_wrapped = new Scalar(170, 70, 50);
        Scalar upper_red_wrapped = new Scalar(180, 255, 255);

        Mat thresh1 = new Mat();
        Core.inRange(hsvMat, lower_red, upper_red, thresh1);

        Mat thresh2 = new Mat();
        Core.inRange(hsvMat, lower_red_wrapped, upper_red_wrapped, thresh2);

        Core.bitwise_or(thresh1, thresh2, threshold);

        return threshold;
    }

    public void findProp(List<MatOfPoint> contours, Mat image) {
        //find hierarchy based on area
        ArrayList<Double> areaList = new ArrayList<>();
        for (MatOfPoint contour : contours) areaList.add(Imgproc.contourArea(contour));

        ArrayList<Double> areaListClone = (ArrayList<Double>) areaList.clone();

        int index = findMaxContourAreaIndex(areaListClone);

        int propIndex = -1;
        if (areaList.size() >= 3) {
            areaListClone.remove(index);
            int index2 = findMaxContourAreaIndex(areaListClone);
            areaListClone.remove(index2);
            int index3 = findMaxContourAreaIndex(areaListClone);

            ArrayList<Integer> indexList = new ArrayList<>();
            indexList.add(Imgproc.boundingRect(contours.get(index)).x);
            indexList.add(Imgproc.boundingRect(contours.get(index2)).x);
            indexList.add(Imgproc.boundingRect(contours.get(index3)).x);

            Collections.sort(indexList);

            propIndex = indexList.indexOf(Imgproc.boundingRect(contours.get(index)).x);
            MatOfPoint propContour = contours.get(index);
            telemetry.addLine("Contour X: " + propContour.toList().get(0).x);
            telemetry.addLine("Contour Y: " + propContour.toList().get(0).y);
            telemetry.addLine("Contour Width: " + propContour.width());
            telemetry.addLine("Contour Height: " + propContour.height());
            telemetry.addLine("Contour Area: " + areaList.get(index));

            Imgproc.rectangle(image, Imgproc.boundingRect(propContour), new Scalar(255, 255, 255));
            Imgproc.drawContours(image, Collections.singletonList(propContour), -1, new Scalar(0, 255, 255), 3);
        }
        position = propIndex == 0 ? PropPosition.LEFT : propIndex == 1 ? PropPosition.MIDDLE :
                propIndex == 2 ? PropPosition.RIGHT : PropPosition.NONE;

        telemetry.addLine("Contours Found: " + contours.size());
        telemetry.addLine("Position: " + position.toString());

        telemetry.update();
    }

    public int findMaxContourAreaIndex(ArrayList<Double> list) {
        double max = 0;
        int index = -1;
        for (int i = 0; i < list.size(); i++) {
            double temp = max;
            max = Math.max(list.get(i), max);
            if (temp != max) {
                index = i;
            }
        }
        return index;
    }

    public PropPosition getPosition() {
        return position;
    }
    public void releaseMats(Mat... mats) {
        for (Mat mat : mats) mat.release();
    }
}
