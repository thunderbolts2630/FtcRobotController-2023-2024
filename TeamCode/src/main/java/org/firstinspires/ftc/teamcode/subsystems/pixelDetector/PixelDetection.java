package org.firstinspires.ftc.teamcode.subsystems.pixelDetector;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.utils.Math.MovingAvrage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
// Import necessary Camera2 API classes

// Import necessary classes


public class PixelDetection implements Subsystem {

    public enum PropPos{
        left,middle,right;

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
    double propAverage=0;
    MovingAvrage filter=new MovingAvrage(10);
    public AllianceColor alliance=AllianceColor.blue;


    public PixelDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
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
            webcam.setPipeline(new RedPipeline(this::updatePos));
        }
        else
        {
            webcam.setPipeline(new BluePipeline(this::updatePos));
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

    public void updatePos(PropPos propPosTemp){
        propAverage=filter.calculate(propPosTemp.ordinal());
        propPos= PixelDetection.PropPos.values()[Math.min(2,(int)Math.round(propAverage))];



        // If no contour is found, print that the object is in the middle side (commit)
        telemetry.addLine("The object is in the "+ propPos.name()+" side");
        dashboardTelemetry.addLine("The object is in the "+ propPos.name()+" side");
        dashboardTelemetry.addData("prop side", propPos.ordinal());
        telemetry.update();
        dashboardTelemetry.update();
    }


}