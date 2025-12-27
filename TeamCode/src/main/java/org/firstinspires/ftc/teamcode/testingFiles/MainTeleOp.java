package org.firstinspires.ftc.teamcode.testingFiles;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.LLstart;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearing;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.close;
//import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.hoodMultClose;
//import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.hoodMultFar;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.far;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.lockSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.mainTag;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.maxBearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeEnd3Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.targetHoodClose;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.targetHoodFar;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.velHoodPos;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kF;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.targetVelocity;

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
    public final int blueTag = 2;
    public final int redTag = 0;
    public int ballFocused = 1;
    private CRServo swivel;
    private DcMotorEx intake1;
    private Limelight3A limelight;
    private boolean locking;
    boolean starte = false;
    boolean hasslept = false;
    double previousError = GlobalVariable.previousError;
    double integralSum = GlobalVariable.integralSum;
    DcMotorEx flywheel;




    @Override
    public void runOpMode() {

        //TODO change end positions near blue/near red
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(GlobalVariable.nearBlue.IntakeEnd3X, GlobalVariable.nearBlue.IntakeEnd3Y, Math.toRadians(0)));

        Push push = new Push(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Index index = new Index(hardwareMap);

        intake1 = hardwareMap.get(DcMotorEx.class, "Intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        //TODO switch tag
        mainTag = redTag;
        swivel = hardwareMap.get(CRServo.class, "Swivel");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.addLine("Initialized");
        telemetry.update();

        Actions.runBlocking(new SequentialAction(index.indexHome(), push.PushBallDown(), hood.hoodPosition()));

        Gamepad cG1 = new Gamepad();
        Gamepad cG2 = new Gamepad();
        Gamepad pG1 = new Gamepad();
        Gamepad pG2 = new Gamepad();

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

            double y = drive.pose.position.y;

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
                close -= y * (200/38);
                shoot(close);
                velHoodPos -=  y * 0.1;
                runningActions.add(new SequentialAction(hood.setHoodPos()));
            } else if (cG2.left_trigger >= 0.1) {
                shoot(far);
                runningActions.add(new SequentialAction(hood.setHoodPos()));
            } else {
                if (LLstart) {
                    limelight.stop();
                    LLstart = false;
                }
            }
            if (cG2.right_trigger <= 0.1 && cG2.left_trigger <= 0.1) {
                flywheelPID(0);
            }
            if (cG2.right_bumper && !pG2.right_bumper) {
                if (cG2.right_trigger >= 0.1) {
                    targetHoodClose += 0.1;
                } else if (cG2.left_trigger >= 0.1) {
                    targetHoodFar += 0.1;
                } else runningActions.add(new SequentialAction(hood.hoodUp()));
                runningActions.add(new SequentialAction(hood.hoodPos()));
            }
            if (cG2.left_bumper && !pG2.left_bumper) {
                if (cG2.right_trigger >= 0.1) {
                    targetHoodClose -= 0.1;
                } else if (cG2.left_trigger >= 0.1) {
                    targetHoodFar -= 0.1;
                } else runningActions.add(new SequentialAction(hood.hoodDown()));
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

            telemetry.addData("Hood Position", hoodPoss);
            telemetry.addData("Target Hood Position", targetHoodClose);
            telemetry.addData("Servo Locked", servoLocked);
            telemetry.addData("detectionsExist", detectionsExist);
            telemetry.addData("flywheel vel", flywheelV);
            telemetry.addData("has slept? ", hasslept);
            telemetry.update();

            drive.updatePoseEstimate();
        }
    }

    void lodk (int tag) {
        if (!LLstart) {
            limelight.start();
            LLstart = true;
        }
        telemetry.addLine("Started");
        limelight.pipelineSwitch(tag);
        LLResult result = limelight.getLatestResult();
        telemetry.addLine("Result?");
        if (result != null) {
            if (result.isValid()) {
                telemetry.addLine("In Loop");
                bearing = result.getTx();
                telemetry.addData("Bearing: ", bearing);
                bearingErr = bearing - maxBearingErr;
                lockSpeed = 0.1 * bearingErr;
                lockSpeed = Math.max(-1, Math.min((lockSpeed), 1));
                if (bearing > maxBearingErr) {
                    swivel.setPower(-lockSpeed);
                } else if (bearing < -maxBearingErr) {
                    swivel.setPower(lockSpeed);
                } else {
                    swivel.setPower(0);
                }
            }
        }
    }

    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }

    void shoot (double distance) {
        close = 600;
        far = 840;
        lodk(mainTag);
        flywheelPID(distance);
        velHoodPos = (flywheel.getVelocity() * (targetHoodClose/distance)); // -0.1 * result.getDistance();
        telemetry.addData("Vel Hood Pos: ", velHoodPos);
    }
}

