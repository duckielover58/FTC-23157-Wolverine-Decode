package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.LLstart;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.ServoLocked;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearing;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.lockSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.mainTag;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.maxBearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.motif;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.postIntake100;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.redTag;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "farPassiveRed", group = "Robot")
public class farPassiveRed extends LinearOpMode {

    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private CRServo swivel;
    private Flywheel flywheel;
    private LimelightCam limelight1;
    private Limelight3A limelight;
    double kP = 0.35;
    double i = 0;
    boolean upies = false;
    int first = 5;
    int offset = 5;
    int second = first + offset;
    int third = second + offset;
    double GPPTag = 21;
    double PGPTag = 22;
    double PPGTag = 23;
    double startXPose = 63.5;
    double startYPose = 8.75;
    double startHPose = Math.toRadians(180);
    boolean servoLocked = false;
    @Override
    public void runOpMode() {

        mainTag = redTag;

       PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(startXPose, startYPose, startHPose));
        Pose2d startPose = new Pose2d(startXPose, startYPose, startHPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        limelight1 = new LimelightCam(hardwareMap);
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Actions.runBlocking(push.PushBallDown());
        Actions.runBlocking(index.outtakeIndex1());
        Actions.runBlocking(hood.hoodPositionInit());
        Actions.runBlocking(flywheel.flywheelInit());

        /*
        while (!servoLocked || !isStarted()) {
      //      lodk(mainTag);
            telemetry.addData("Locking: ", servoLocked);
            telemetry.update();
        }

         */

        limelight.stop();
        telemetry.addLine("Locked");
        telemetry.update();

        waitForStart();
        limelight.stop();
        Actions.runBlocking(limelight1.motifCheck());

        if (isStopRequested()) return;

        Action shootThreeBalls21 = drive.actionBuilder(new Pose2d(startXPose, startYPose, startHPose))
                .waitSeconds(5)
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> ServoLocked = false),
                        hood.ten(),
                        index.outtakeIndex1(),
                        new SleepAction(0.2),
                        push.PushBallUp(),

                        hood.eight(),
                        new SleepAction(0.25),
                        push.PushBallDown(),
                        new SleepAction(0.2),
                        index.outtakeIndex2(),
                        new SleepAction(0.55),
                        push.PushBallUp(),

                        hood.six(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.1),
                        index.outtakeIndex3(),
                        new SleepAction(0.65),
                        push.PushBallUp(),

                        new SleepAction(0.2),
                        push.PushBallDown(),
                        hood.ten(),
                        new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                        index.intakeIndex1(),
                        new InstantAction(() -> ServoLocked = true)
                ))
                .stopAndAdd(new InstantAction(() -> postIntake100 = false))
                .build();

        Action shootThreeBalls22 = drive.actionBuilder(new Pose2d(startXPose, startYPose, startHPose))
                .waitSeconds(5)
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> ServoLocked = false),
                        hood.ten(),
                        index.outtakeIndex2(),
                        new SleepAction(0.2),
                        push.PushBallUp(),

                        hood.eight(),
                        new SleepAction(0.25),
                        push.PushBallDown(),
                        new SleepAction(0.2),
                        index.outtakeIndex1(),
                        new SleepAction(0.55),
                        push.PushBallUp(),

                        hood.six(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.1),
                        index.outtakeIndex3(),
                        new SleepAction(0.65),
                        push.PushBallUp(),

                        new SleepAction(0.2),
                        push.PushBallDown(),
                        hood.ten(),
                        new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                        index.intakeIndex1(),
                        new InstantAction(() -> ServoLocked = true)
                        ))
                .stopAndAdd(new InstantAction(() -> postIntake100 = false))
                .build();

        Action shootThreeBalls23 = drive.actionBuilder(new Pose2d(startXPose, startYPose, startHPose))
                .waitSeconds(5)
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> ServoLocked = false),
                        hood.ten(),
                        index.outtakeIndex3(),
                        new SleepAction(0.2),
                        push.PushBallUp(),

                        hood.eight(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.2),
                        index.outtakeIndex2(),
                        new SleepAction(0.55),
                        push.PushBallUp(),

                        hood.six(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.1),
                        index.outtakeIndex1(),
                        new SleepAction(0.65),
                        push.PushBallUp(),

                        new SleepAction(0.2),
                        push.PushBallDown(),
                        hood.ten(),
                        new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                        index.intakeIndex1(),
                        new InstantAction(() -> ServoLocked = true)
                        ))
                .stopAndAdd(new InstantAction(() -> postIntake100 = false))
                .build();


        Action farPassive = drive.actionBuilder(new Pose2d(startXPose, startYPose, Math.toRadians(165)))
                .strafeToLinearHeading(new Vector2d(37.5, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(63)
                .waitSeconds(0.5)
//                .splineToLinearHeading(new Pose2d(((24*3)-35), 63, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallStop())
                .strafeToLinearHeading(new Vector2d(startXPose, startYPose), Math.toRadians(165))
                .build();
        Action startHeadingChange = drive.actionBuilder(new Pose2d(startXPose,startYPose,startHPose))
                .strafeToLinearHeading(new Vector2d(57.5,9.75), Math.toRadians(165))
                .build();
        if (motif == 21) {
            Actions.runBlocking(new SequentialAction(
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            startHeadingChange,
                            shootThreeBalls21,
                            flywheel.PIDp2()
                    ),
                    farPassive,
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            shootThreeBalls21,
                            flywheel.PIDp2()
                    )
            ));
        } else if (motif == 22) {
            Actions.runBlocking(new SequentialAction(
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            startHeadingChange,
                            shootThreeBalls22,
                            flywheel.PIDp2()
                    ),
                    farPassive,
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            shootThreeBalls22,
                            flywheel.PIDp2()
                    )
            ));
        } else {
            Actions.runBlocking(new SequentialAction(
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            startHeadingChange,
                            shootThreeBalls23,
                            flywheel.PIDp2()
                    ),
                    farPassive,
                    new InstantAction(() -> postIntake100 = true),
                    new ParallelAction(
                            shootThreeBalls23,
                            flywheel.PIDp2()
                    )
            ));
        }
    }

 /*   void lodk (int tag) {
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
                bearingErr = bearing - maxBearingErr;
                lockSpeed = 0.1 * bearingErr;
                lockSpeed = Math.max(-0.17, Math.min((lockSpeed), 0.17));
                if (bearing > -3) {
                    swivel.setDirection(CRServo.Direction.FORWARD);
                    swivel.setPower(-lockSpeed);
                } else if (bearing < -7) {
                    swivel.setDirection(CRServo.Direction.REVERSE);
                    swivel.setPower(lockSpeed);
                } else {
                    swivel.setPower(0);
                    servoLocked = true;
                }
            } else swivel.setPower(0);
        } else swivel.setPower(0);
    }
    */
}