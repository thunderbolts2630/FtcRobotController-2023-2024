package org.firstinspires.ftc.teamcode.subsystems.pixelDetector;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.subsystems.pixelDetector.BluePipeline.pipelineConfigBlue.*;
class BluePipeline extends OpenCvPipeline {

    @Config
    public static class pipelineConfigBlue {
        public static int show=0;
        public static int preD=20,dilation=22, erosion =35;
        public static int area_max=32000,area_min=6000;
        public static int left_ROI_min=20,left_ROI_max=320;
        public static int right_ROI_min=960,right_ROI_max=1280;
        public static int roi_top=220,roi_bottom=440;
        public static double lH = 0, lS = 0, lV =50;
        public static double hH = 1, hS = 1, hV = 95;
    }

    Mat hsv = new Mat();
    Mat mask= new Mat();
    Mat dilated=new Mat();
    Mat eroded=new Mat();
    Mat step3= new Mat();
    Scalar lowerColor = new Scalar(lH, lS, lV);
    Scalar upperColor = new Scalar(hH, hS, hV);
    Mat leftRoiPostFilter = new Mat();
    Mat rightRoiPostFilter =new Mat();
    Mat hierarchy = new Mat();
    Telemetry dashboardTelemetry= FtcDashboard.getInstance().getTelemetry();
    Consumer<PixelDetection.PropPos> updatePos;

    public BluePipeline(Consumer<PixelDetection.PropPos> updatePos) {
        super();
        this.updatePos = updatePos;
    }

    public Rect FindPropInROI(Mat roi, PixelDetection.PropPos id) {

        Imgproc.cvtColor(roi, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, lowerColor, upperColor, mask);
        Imgproc.dilate(mask,dilated,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(preD,preD)));
        Imgproc.erode(dilated,eroded,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(erosion, erosion)));
        Imgproc.dilate(eroded,step3,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(dilation,dilation)));

        if(PixelDetection.PropPos.right==id){
            rightRoiPostFilter =step3.clone();
        }
        else {
            leftRoiPostFilter =step3.clone();
        }

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double largestArea = 0;
        Rect largestRect = null;

        for (MatOfPoint contour : contours) {
            double area =Imgproc.contourArea(contour);
            dashboardTelemetry.addData("area"+id,area);

            // Set a threshold for the minimum area to filter small contours
            if (area >area_min && area < area_max) {
                if (area > largestArea) {
                    largestArea = area;
                    largestRect = Imgproc.boundingRect(contour);
                }
            }
        }
        dashboardTelemetry.addData("largest area"+id,largestArea);
        return largestRect;

    }

    @Override
    public Mat processFrame(Mat input) {
//
            /* for calib

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            mask = new Mat();
            Core.inRange(hsv, lowerColor, upperColor, mask);
            dilated=new Mat();
            Imgproc.dilate(mask,dilated,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(3,3)));
            return dilated;
            */
//            // the above is for calib
        lowerColor = new Scalar(lH, lS, lV);
        upperColor = new Scalar(hH, hS, hV);

        Mat roiLeft;
        Mat roiRight;


        Range rowRange = new Range(roi_top, roi_bottom);
        Range colRangeLeft = new Range(left_ROI_min, left_ROI_max);
        Range colRangeRight = new Range(right_ROI_min, right_ROI_max);
        dashboardTelemetry.addData("mat cols",input.cols());
        dashboardTelemetry.addData("mat rows",input.rows());
        dashboardTelemetry.update();

        roiLeft = new Mat(input, rowRange, colRangeLeft);
        roiRight = new Mat(input,rowRange, colRangeRight);

        Rect rectLeft = FindPropInROI(roiLeft, PixelDetection.PropPos.left);
        Rect rectRight = FindPropInROI(roiRight, PixelDetection.PropPos.right);

        if (rectLeft != null) {
            updatePos.accept( PixelDetection.PropPos.left);
        } else {
            if (rectRight != null) {
                updatePos.accept(PixelDetection.PropPos.right);
            }else {
                updatePos.accept(PixelDetection.PropPos.middle);
            }
        }

        return show==0?input:leftRoiPostFilter;

    }
}
