package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.testingFiles.AprilTagGameDatabaseEditable;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Date;
import java.util.List;

public class Camera {
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.5, 1, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double robotHeading = 0; //placeholder
    private double cameraHeading = robotHeading;
    public static double ServoPower = 0;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;


    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initAprilTag();
    }


    private void initAprilTag() {
        telemetry.addData("Initializing Camera...", aprilTag);
        // Create the AprilTag processor.
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabaseEditable.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(cam);
        builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }
    private double bearing;
    private boolean telemetryAprilTag(int aprilTag, int maxWaitSec) throws InterruptedException {
        Swivel swivel = new Swivel(hardwareMap);
        long startTimeMillis = new Date().getTime();
        int timeDeltaSec = 0;

        telemetry.addData("starting turret lock While...", aprilTag);
        while (Math.abs(bearing) <= 1 && timeDeltaSec < maxWaitSec) {
        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == aprilTag) {
                double sped = 0.1;
                if (detection.ftcPose.bearing > 1) {
                    ServoPower = -sped;
                    bearing = detection.ftcPose.bearing;
                    telemetry.addData("Bearing", bearing);
                    Actions.runBlocking(swivel.aim());

                } else if (detection.ftcPose.bearing < -1) {
                    ServoPower = sped;
                    bearing = detection.ftcPose.bearing;
                    telemetry.addData("Bearing", bearing);
                    Actions.runBlocking(swivel.aim());
                } else if (Math.abs(detection.ftcPose.bearing) <= 1) {
                    ServoPower = 0;
                    bearing = detection.ftcPose.bearing;
                    telemetry.addData("Bearing", bearing);
                    Actions.runBlocking(swivel.aim());
                    telemetry.addData("Bearing reached within limit", bearing);
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addData("Servo Power:", ServoPower);
                telemetry.addData("Bearing:", detection.ftcPose.bearing);

            } else {
                telemetry.addData("Tag scanned NOT matching !!!! ", detection.id);
            }

            // Add "key" information to telemetry
            telemetry.addData("Servo Power:", ServoPower);
            telemetry.addData("Bearing:", bearing);
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

            Thread.sleep(20);
            }
            timeDeltaSec = (int)(new Date().getTime() - startTimeMillis)/1000;
        }
        return false;
    }

    public class CamLock implements Action{
        int aprilTag;

        public CamLock(int aprilTag) {
            this.aprilTag = aprilTag;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            try {
                telemetry.addData("starting turret lock ", aprilTag);
                telemetryPacket.addLine("starting turret lock ");
                boolean camLockResult = telemetryAprilTag(aprilTag, 5);
                telemetry.addData("Camera lock for Tag ", camLockResult);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }
    public Action getCamLock (int aprilTag){
        telemetry.addData("Adding Camera for ", aprilTag);
        return new CamLock(aprilTag);
    }

}


