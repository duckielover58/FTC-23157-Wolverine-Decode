package org.firstinspires.ftc.teamcode.testingFiles;

import static org.firstinspires.ftc.teamcode.testingFiles.ShooterLockTest.bearing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    // Non-blocking lock state
    boolean lockActive = false;
    int lockTarget1 = 0;
    int lockTarget2 = 0;
    long lastSeenTime = 0;
    final long LOST_TIMEOUT_MS = 300;  // stop lock if no tag for 0.3 sec

    public boolean servoLocked = false;
    public boolean detectionsExist = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.5, 1, 0);
    //TODO Measure with tape
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private double robotHeading = 0; //placeholder
    private double cameraHeading = robotHeading;
    public static double ServoPower = 0;
    public final int blueTag = 20;
    public final int redTag = 24;
    public int ballFocused = 1;
    private CRServo swivel;

    @Override
    public void runOpMode() {

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        Intake intake = new Intake(hardwareMap);
        Push push = new Push(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        DcMotor flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");
        FlywheelController flyCtrl = new FlywheelController(flywheelMotor);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        //Swivel swivel = new Swivel(hardwareMap);
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        Index index = new Index(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));

            drive.updatePoseEstimate();

            if (!currentGamepad1.right_bumper && previousGamepad1.left_bumper) {
                runningActions.add(new SequentialAction(intake.IntakeBall()));
            } else {
                runningActions.add(new SequentialAction(intake.IntakeBallStop()));
            }
            if (!currentGamepad1.left_bumper && previousGamepad1.right_bumper) {
                runningActions.add(new SequentialAction(intake.IntakeBallReverse()));
            } else {
                runningActions.add(new SequentialAction(intake.IntakeBallStop()));
            }
            if (!currentGamepad2.dpad_up && previousGamepad2.dpad_up) {
                runningActions.add(new SequentialAction(push.PushBallUp()));
            }
            if (!currentGamepad2.dpad_down && previousGamepad2.dpad_down) {
                runningActions.add(new SequentialAction(push.PushBallDown()));
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.left_bumper) {
                runningActions.add(new SequentialAction(flywheel.shoot()));
            } else {
                runningActions.add(new SequentialAction(flywheel.shootStop()));
            }
            if (!currentGamepad2.y && previousGamepad2.y) {
                if (ballFocused == 1) {
                    Actions.runBlocking(index.index2());
                    ballFocused = 2;
                } else if (ballFocused == 2) {
                    Actions.runBlocking(index.index3());
                    ballFocused = 3;
                } else {
                    Actions.runBlocking(index.index1());
                    ballFocused = 1;

                }
            }
            if (!currentGamepad2.b && previousGamepad2.b) {
                if (ballFocused == 3) {
                    runningActions.add(new SequentialAction(index.index2()));
                    ballFocused = 2;
                } else if (ballFocused == 1) {
                    runningActions.add(new SequentialAction(index.index3()));
                    ballFocused = 3;
                } else {
                    runningActions.add(new SequentialAction(index.index1()));
                    ballFocused = 1;
                }
            }
            if (gamepad2.a) {
                servoLocked = false;
                lock(redTag, blueTag);
            }
            /*
            if (gamepad1.dpad_right) {
                Actions.runBlocking(swivel.aim());
            }

             */

            telemetry.addData("Servo Locked", servoLocked);
            telemetry.addData("detectionsExist", detectionsExist);
            telemetry.update();
        }
    }

    private void lock(int tag1, int tag2) {
        initAprilTag();
        telemetryAprilTag(tag1, tag2);
        while (!servoLocked && detectionsExist) {
            telemetryAprilTag(tag1, tag2);
            telemetry.update();
            ServoPower = 0;
            swivel.setPower(ServoPower);
            sleep(20);
        }

        detectionsExist = true;

        visionPortal.close();
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //Cam visual settings
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabaseEditable.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        //Cam fps
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        sleep(100);
        telemetry.addData("On Init: # AprilTags Detected", currentDetections.size());
        if (currentDetections.isEmpty()) {
            detectionsExist = false;
        } else {
            detectionsExist = true;
        }
    }
    private void telemetryAprilTag(int tag1, int tag2) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
        if (detection.id == tag1 || detection.id == tag2) {
            if (detection.id != 0) {
                double sped = 0.1;
                double bearingErr = 1;

                if (detection.ftcPose.bearing > bearingErr + 3) {
                    ServoPower = -sped;
                    telemetry.addData("Bearing", bearing);
                    swivel.setPower(ServoPower);

                } else if (detection.ftcPose.bearing < -bearingErr + 3) {
                    ServoPower = sped;
                    telemetry.addData("Bearing", bearing);
                    swivel.setPower(ServoPower);
                } else if (-bearingErr + 3 <= detection.ftcPose.bearing && detection.ftcPose.bearing <= bearingErr + 3) {
                    ServoPower = 0;
                    telemetry.addData("Bearing", bearing);
                    swivel.setPower(ServoPower);
                    telemetry.addData("Bearing reached within limit", bearing);
                    servoLocked = true;
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addData("Servo Power:", ServoPower);
                telemetry.addData("Bearing:", detection.ftcPose.bearing);

            }
        }

            // Add "key" information to telemetry
            telemetry.addData("Servo Power:", ServoPower);
            telemetry.addData("Bearing:", bearing);
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

            sleep(20);
        }
    }



}

