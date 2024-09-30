package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lib.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AutoDetectAprilTag extends CommandBase {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int cameraMonitorViewId;
    double m_timeOut;
    ElapsedTime m_elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    AprilTagDetection tagOfInterest = null;


    public AutoDetectAprilTag(double _timeOut){
        m_timeOut = _timeOut;
    }
    @Override
    public void initialize() {
        cameraMonitorViewId = g.ROBOT.OpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", g.ROBOT.OpMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(g.ROBOT.OpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(g.CAMERA.TAG_SIZE, g.CAMERA.FX, g.CAMERA.FY, g.CAMERA.CX, g.CAMERA.CY);

        camera.setPipeline(aprilTagDetectionPipeline);
        // NOTE: this must be called *before* you call startStreaming(...)
        //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        m_elapsedTimer.reset();
    }
    @Override
    public void execute() {
        //        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == g.CAMERA.TagOfInterest) {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
        g.ROBOT.OpMode.sleep(20);

        if (tagOfInterest != null) { // We found a tag so set the GlobalVariables
            // Set the global variable numbers
            g.CAMERA.AprilTag_X = tagOfInterest.pose.x * 1000;
            g.CAMERA.AprilTag_Y = tagOfInterest.pose.y * 1000;
            g.CAMERA.AprilTag_Z = tagOfInterest.pose.z * 1000 * 1.25;
            //GlobalData.tagPoseZ = tagOfInterest.pose.z;
            g.CAMERA.AprilTagBearing = Math.toDegrees(Math.atan(g.CAMERA.AprilTag_X / g.CAMERA.AprilTag_Z));
            g.CAMERA.AprilTagRange = Math.sqrt(g.CAMERA.AprilTag_Z * g.CAMERA.AprilTag_Z + g.CAMERA.AprilTag_X * g.CAMERA.AprilTag_X);

        } else { // Did not find the tag so add some default data.
            g.CAMERA.AprilTag_X = 0;
            g.CAMERA.AprilTag_Y = 0;
            g.CAMERA.AprilTag_Z = 0;
            g.CAMERA.AprilTagBearing = 5;
            g.CAMERA.AprilTagRange = 150;
        }
    }
    @Override
    public boolean isFinished() {

        return tagOfInterest != null && m_elapsedTimer.seconds() > m_timeOut;
    }
}
