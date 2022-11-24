package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.AndroidUtils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

public class CameraSystem {
    private final LinearOpMode opMode;
    private final CameraPipeline cameraPipeline;
    private final OpenCvCamera camera;

    // -- AprilTag related variables -- //
    // camera characteristics
    static final double FX = 578.272;
    static final double FY = 578.272;
    static final double CX = 402.145;
    static final double CY = 221.506;
    static final double TAG_SIZE = 0.166; // UNITS ARE METERS

    // types for AprilTag IDs
    public enum AprilTagType {
        TAG_1,
        TAG_2,
        TAG_3,
        UNIDENTIFIED,
        DETECTION_ERROR,
        INVALID,
        DETECTION_IN_PROGRESS
    }

    // inline camera pipeline class
    static class CameraPipeline extends OpenCvPipeline {
        private long nativeApriltagPtr; // a pointer to the JNI AprilTag detector
        private final Mat grey = new Mat(); // a blank canvas to store the b&w image

        private final LinearOpMode opMode; // the OpMode that's currently running
        private boolean isCapturingImage = false;
        private boolean isDetectingAprilTag = false;

        public AprilTagType aprilTagID = AprilTagType.UNIDENTIFIED; // last identified AprilTag

        public CameraPipeline(LinearOpMode opMode) {
            this.opMode = opMode;

            // initiate AprilTag detector
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        // runs when class is deleted
        @Override
        protected void finalize() {
            // -- safely delete AprilTag detector from memory -- //
            // Might be null if createApriltagDetector() threw an exception
            if (nativeApriltagPtr != 0) {
                // Delete the native context we created in the constructor
//                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            } else {
                // handle exceptions
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        /**
         * Capture an image and save to robot
         */
        public void captureImage() {
            isCapturingImage = true;
        }

        /**
         * Run AprilTag detection
         */
        public void detectAprilTag() {
            isDetectingAprilTag = true;
        }

        // runs automatically every frame
        @Override
        public Mat processFrame(Mat input) {
            // image capturing is enabled
            if (isCapturingImage) {
                opMode.telemetry.addLine("Capture");
                opMode.telemetry.update();

                isCapturingImage = false; // turn off capturing an image for the next iteration

                try {
                    String timeStamp = AndroidUtils.timestampString(); // get current time
                    String filepath = // target path for the image capture
                            new File(AppUtil.ROBOT_DATA_DIR, String.format("img_%s.png", timeStamp)).getAbsolutePath();
                    // save capture to disk
                    saveMatToDiskFullPath(input, filepath);

                    opMode.telemetry.addLine("Capturing Image");
                    opMode.telemetry.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            // AprilTag detection is enabled
            if (isDetectingAprilTag) {
                isDetectingAprilTag = false; // turn off AprilTag detection for the next iteration

                try {
                    // convert image to grey
                    Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

                    // call the detector on the image
                    ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, TAG_SIZE, FX, FY, CX, CY);

                    if (detections.size() > 0) { // detected at least one april tag
                        // assign global variable based on result
                        switch (detections.get(0).id) {
                            case 0:
                                aprilTagID = AprilTagType.TAG_1;
                                break;
                            case 2:
                                aprilTagID = AprilTagType.TAG_2;
                                break;
                            case 3:
                                aprilTagID = AprilTagType.TAG_3;
                                break;
                            default:
                                aprilTagID = AprilTagType.INVALID;
                        }
                    } else { // no AprilTag detected
                        aprilTagID = AprilTagType.DETECTION_ERROR;
                    }
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
            return input;
        }
    }

    public CameraSystem(LinearOpMode opMode) {
        this.opMode = opMode;

        cameraPipeline = new CameraPipeline(opMode); // initiate camera pipeline
        int cameraMonitorViewId =
                this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        WebcamName webcamName =
                this.opMode.hardwareMap.get(WebcamName.class, "webcam");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(cameraPipeline);
        camera.openCameraDevice();
        camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * Take an image capture
     */
    public void captureImage() {
        cameraPipeline.captureImage();
    }

    /**
     * Detects AprilTag from camera
     */
    public AprilTagType detectAprilTag() {
        // reset previous april tag id
        cameraPipeline.aprilTagID = AprilTagType.DETECTION_IN_PROGRESS;

        // make the processFrame loop try to detect AprilTags
        cameraPipeline.detectAprilTag();

        // block the main loop until an AprilTag is detected
        while (opMode.opModeIsActive() && cameraPipeline.aprilTagID == AprilTagType.DETECTION_IN_PROGRESS) {
        }

        // return the id
        return cameraPipeline.aprilTagID;
    }
}
