package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.AndroidUtils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

public class CameraSystem {
    private final LinearOpMode opMode;
    private final CameraPipeline cameraPipeline;
    private final OpenCvCamera camera;

    static class CameraPipeline extends OpenCvPipeline {
        private boolean isCapturingImage = false;
        private LinearOpMode opMode;

        public CameraPipeline(LinearOpMode opMode) {
            this.opMode = opMode;
        }

        public void captureImage() {
            isCapturingImage = true;
        }

        @Override
        public Mat processFrame(Mat input) {
            if (isCapturingImage) {
                isCapturingImage = false;
                try {
                    String timeStamp = AndroidUtils.timestampString();
                    String filepath = new File(AppUtil.ROBOT_DATA_DIR, String.format("img_%s.png", timeStamp)).getAbsolutePath();
                    saveMatToDiskFullPath(input, filepath);
                    opMode.telemetry.addLine("Capturing Image");
                    opMode.telemetry.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return input;
        }
    }

    public CameraSystem(LinearOpMode opMode) {
        cameraPipeline = new CameraPipeline(opMode);
        this.opMode = opMode;
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        WebcamName webcamName = this.opMode.hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(cameraPipeline);
        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void captureImage(){
        cameraPipeline.captureImage();
    }


}
