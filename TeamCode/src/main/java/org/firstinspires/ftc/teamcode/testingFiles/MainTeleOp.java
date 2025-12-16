package org.firstinspires.ftc.teamcode.testingFiles;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.previousTime;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kD;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kI;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystems.GlobalVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    // Non-blocking lock state
    boolean lockActive = false;
    int lockTarget1 = 0;
    int lockTarget2 = 0;
    long lastSeenTime = 0;
    final long LOST_TIMEOUT_MS = 300;  // stop lock if no tag for 0.3 sec
    ExposureControl exposureControl;
    private int     myExposure  = 1420;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain    =   30;
    private int     minGain ;
    private int     maxGain ;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;

    public boolean servoLocked = true;
    public boolean detectionsExist = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.5, 3, 0);
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
    public static double ServoPos = 0;
    public static double flywheelV;
    public static double hoodPoss;
    public final int blueTag = 20;
    public final int redTag = 24;
    public int ballFocused = 1;
    private CRServo swivel;
    private DcMotorEx intake1;
//    private Limelight3A limelight;
    private boolean locking;
    boolean starte = false;
    boolean hasslept = false;
    double previousError = GlobalVariable.previousError;
    double integralSum = GlobalVariable.integralSum;
    DcMotorEx flywheel;




    @Override
    public void runOpMode() {

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        Intake intake = new Intake(hardwareMap);
        intake1 = hardwareMap.get(DcMotorEx.class, "Intake");
        Push push = new Push(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hood hood = new Hood(hardwareMap);
        Gamepad cG1 = new Gamepad();
        Gamepad cG2 = new Gamepad();
        Gamepad pG1 = new Gamepad();
        Gamepad pG2 = new Gamepad();
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        Index index = new Index(hardwareMap);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Initialized");
        telemetry.update();

        class ShootThreeBalls implements Action {
            private final Action sequence;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return sequence.run(packet);
            }

            public ShootThreeBalls() {
                sequence = new SequentialAction(
                        index.index2(),
                        new InstantAction(() -> flywheelPID(700)),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(2.5),
                        push.PushBallUp(),
                        new SleepAction(0.3),
                        push.PushBallDown(),
                        new SleepAction(0.5),
                        index.index3(),
                        new SleepAction(0.80),
                        push.PushBallUp(),
                        new SleepAction(0.3),
                        push.PushBallDown(),
                        new SleepAction(0.5),
                        index.index1(),
                        new SleepAction(0.7),
                        push.PushBallUp(),
                        new SleepAction(0.3),
                        push.PushBallDown(),
                        new SleepAction(0.5),
                        new InstantAction(() -> flywheelPID(0)),
                        new SleepAction(0.2),
                        index.index1()
                );
            }
        }



        Actions.runBlocking(new SequentialAction(index.indexHome(), push.PushBallDown(), hood.hoodPosition()));

        waitForStart();

        while (opModeIsActive()) {

            sleep(10);

            pG1.copy(cG1);
            pG2.copy(cG2);

            cG1.copy(gamepad1);
            cG2.copy(gamepad2);

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

            if (cG1.right_trigger >= 0.1) {
                intake1.setPower(1);
            } else {
                intake1.setPower(0);
            }
            if (cG1.left_trigger >= 0.1) {
                intake1.setPower(-1);
            }
            if (!cG2.dpad_up && pG2.dpad_up) {
                Actions.runBlocking(push.PushBallUp());
                sleep(300);
                Actions.runBlocking(push.PushBallDown());
            }
            if (cG2.right_trigger >= 0.1) {
                flywheelPID(GlobalVariable.close);
            }
            if (cG2.left_trigger >= 0.1) {
                flywheelPID(GlobalVariable.far);
            }
            if (cG2.right_trigger <= 0.1 && cG2.left_trigger <= 0.1) {
                flywheelPID(0);
            }
            if (cG2.right_bumper && !pG2.right_bumper) {
                runningActions.add(new SequentialAction(hood.hoodUp()));
                runningActions.add(new SequentialAction(hood.hoodPos()));
            }
            if (cG2.left_bumper && !pG2.left_bumper) {
                runningActions.add(new SequentialAction(hood.hoodDown()));
                runningActions.add(new SequentialAction(hood.hoodPos()));
            }
            if (!cG2.y && pG2.y) {
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
            if (!cG2.b && pG2.b) {
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
            if (cG1.a && !pG1.a) {
                //limelightInits();
//                servoLocked = false;
//                lock(redTag, blueTag);
            }
            if (cG1.x && !pG1.x) {
                runningActions.add(new ShootThreeBalls());
            }
//            lodk();

            telemetry.addData("Hood Position", hoodPoss);
            telemetry.addData("Servo Locked", servoLocked);
            telemetry.addData("detectionsExist", detectionsExist);
            telemetry.addData("flywheel vel", flywheelV);
            telemetry.addData("has slept? ", hasslept);
            telemetry.update();
        }
    }
    /*

    void limelightInits() {
        servoLocked = false;
        hasslept = false;
        telemetry.addLine("limelight started");
        if (starte = false) {
            limelight.start();
        } else {
            limelight.stop();
        }
        limelight.pipelineSwitch(redTag);
        telemetry.addLine("limelight pipeline switched");
    }
    void lodk () {
        if (!servoLocked) {
            LLResult result = limelight.getLatestResult();
            telemetry.addLine("result");
            if (!hasslept) {
                sleep(110);
                hasslept = true;
            }
            if (result != null && result.isValid()) {
                telemetry.addLine("in loop");

                double bearing = result.getTx(); // x offset in degrees from target (target x and bearing are the same thing i think)

                // servo aiming/locking
                double bearingThreshold = 3;
                double servoSpeed = 1;

                if (bearing > bearingThreshold) {
                    telemetry.addLine("adjusting swivel");
                    ServoPower = -servoSpeed;
                } else if (bearing < -bearingThreshold) {
                    telemetry.addLine("adjusting swivel");
                    ServoPower = servoSpeed;
                } else {
                    telemetry.addLine("stopping swivel");
                    ServoPower = 0;
                    servoLocked = true;
                }

                swivel.setPower(ServoPower);

                // telemetry
                telemetry.addData("Bearing", bearing);
                telemetry.addData("Servo Power", ServoPower);
                telemetry.addData("Servo Locked", servoLocked);
                telemetry.addData("Target Valid", result.isValid());
            }
        }
    }

     */



    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double currentTime = getRuntime();
        double dt = currentTime - previousTime;
        double error = target - currentVelocity;

        integralSum += (error * dt);
        double derivative = (error - previousError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Math.max(-1.0, Math.min(1.0, output));

        flywheel.setPower(output);

        telemetry.addData("Target Velocity: ", target);
        telemetry.addData("Current Velocity: ", currentVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Power: ", output);

        previousError = error;
        previousTime = currentTime;
    }

    private void lock(int tag1, int tag2) {
        initAprilTag();
        getCameraSetting();
        setManualExposure(myExposure, myGain);
        telemetryAprilTag(tag1, tag2);
        while (!servoLocked) {
            telemetryAprilTag(tag1, tag2);
            telemetry.update();
            sleep(20);
        }

        if (servoLocked || !detectionsExist) {
            ServoPower = 0;
            swivel.setPower(ServoPower);
        }

        detectionsExist = true;

        visionPortal.close();
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .setTagLibrary(AprilTagGameDatabaseEditable.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
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
    public double bearing;
    private void telemetryAprilTag(int tag1, int tag2) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        sleep(100);
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tag1 || detection.id == tag2) {
                double sped = 0.1;
                if (detection.ftcPose.bearing > 1) {
                    ServoPower = -sped;
                    telemetry.addData("Bearing", bearing);
                    swivel.setPower(ServoPower);

                } else if (detection.ftcPose.bearing < -1) {
                    ServoPower = sped;
                    telemetry.addData("Bearing", bearing);
                    swivel.setPower(ServoPower);

                } else {
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

            // Add "key" information to telemetry
            telemetry.addData("Servo Power:", ServoPower);
            telemetry.addData("Bearing:", bearing);
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

            sleep(20);
        }
    }
    private boolean setManualExposure (int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}

