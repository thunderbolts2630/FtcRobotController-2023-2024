package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.PixelDetection.pipelineConfigBlue.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.utils.Math.MovingAvrage;
import org.opencv.core.Range;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
// Import necessary Camera2 API classes

// Import necessary classes


public class PixelDetection implements Subsystem {
    public enum PropPos{
        left(350   ,800),middle(0,0),right(350,1000);
        int maxArea;
        int minArea;

        PropPos(int maxArea, int minArea) {
            this.maxArea = maxArea;
            this.minArea = minArea;
        }
    }

    public enum AllianceColor{
        red,blue;
    }
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvWebcam webcam;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public PropPos propPos=PropPos.middle;
    PropPos propPosTemp;
    double propAverage;
    MovingAvrage filter;
    public AllianceColor alliance=AllianceColor.blue;


    public PixelDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        filter=new MovingAvrage(10);
        propAverage=0;
        register();
    }

    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "camera"),
                        cameraMonitorViewId);
        if(alliance==AllianceColor.red) {
            webcam.setPipeline(new ColorDetectionPipelineBlue());
        }
        else
        {
            webcam.setPipeline(new ColorDetectionPipelineBlue());
        }
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                webcam.getExposureControl().setMode(ExposureControl.Mode.Auto);
//                boolean succes =webcam.getExposureControl().setExposure(Constants.WebcamExposure.webcamExposure, TimeUnit.MICROSECONDS);
//                webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
//                dashboardTelemetry.addData("exposure succes",succes?1:0);
                dashboardTelemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void closeCamera(){
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                dashboardTelemetry.addData("closed",10);
            }
        });
    }
    @Override
    public void periodic() {
    }

    
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


    class ColorDetectionPipelineBlue extends OpenCvPipeline {

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


        public Rect FindPropInROI(Mat roi, PropPos id) {

            Imgproc.cvtColor(roi, hsv, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsv, lowerColor, upperColor, mask);
            Imgproc.dilate(mask,dilated,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(preD,preD)));
            Imgproc.erode(dilated,eroded,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(erosion, erosion)));
            Imgproc.dilate(eroded,step3,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(dilation,dilation)));

            if(PropPos.right==id){
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
                if (area >area_min && area < area_max) { // You can adjust this threshold based on your needs
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
            propPosTemp = PropPos.middle;
            Rect largestRect = null;

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

            Rect rectLeft = FindPropInROI(roiLeft, PropPos.left);
            Rect rectRight = FindPropInROI(roiRight, PropPos.right);

            if (rectLeft != null) {
                propPosTemp =PropPos.left;
                largestRect = rectLeft;
            } else {
                if (rectRight != null) {
                    propPosTemp =PropPos.right;
                    largestRect=rectRight;
                }
            }
            propAverage=filter.calculate(propPosTemp.ordinal());
            propPos=PropPos.values()[Math.min(2,(int)Math.round(propAverage))];

            if (largestRect != null) {
                largestRect.y+=(int)(input.height()*(2.0/3.0));
                if(propPosTemp ==PropPos.right){
                    largestRect.x+=(int)(input.width()*(2.0/3.0));
                }
                Imgproc.rectangle(input, largestRect, new Scalar(0, 255, 0), 2);

                int x = largestRect.x;
                int y = largestRect.y;
                int w = largestRect.width;
                int h = largestRect.height;

                // You can print or store the location as needed
                dashboardTelemetry.addData("prop area",largestRect.area());
            }


            // If no contour is found, print that the object is in the middle side (commit)
            telemetry.addLine("The object is in the "+ propPos.name()+" side");
            dashboardTelemetry.addLine("The object is in the "+ propPos.name()+" side");
            dashboardTelemetry.addData("prop side", propPos.ordinal());
            telemetry.update();
            dashboardTelemetry.update();
            return show==0?input:leftRoiPostFilter;

        }
    }


}