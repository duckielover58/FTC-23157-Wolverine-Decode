package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.*;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.postIntake100;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "closePassiveBlueNine", group = "Robot")
public class closePassiveBlueNine extends LinearOpMode {

    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;
    private Flywheel flywheel1;

    double first = 8;
    double offset = 4.5;
    double second = first + offset;
    double third = second + offset;

    private class ShootThreeBalls2 implements Action {
        private final Action sequence;
        public ShootThreeBalls2() {
            sequence = new SequentialAction(
                    hood.ten(),
                    index.outtakeIndex1(),
                    push.PushBallUp(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex2(),
                    new SleepAction(0.15),
                    push.PushBallUp(),
                    hood.ninefive(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex3(),
                    new SleepAction(0.15),
                    push.PushBallUp(),
                    hood.ten(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.05),
                    index.intakeIndex1()
            );
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(startX, startY, startH);

        Vector2d shootVector = new Vector2d(-11, -11);
        double shootHeading = Math.toRadians(-125);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel1 = new Flywheel(hardwareMap);

        Actions.runBlocking(index.outtakeIndex1());
        Actions.runBlocking(push.PushBallDown());
        Actions.runBlocking(hood.ten());
        Actions.runBlocking(flywheel1.flywheelInit());

        waitForStart();
        if (isStopRequested()) return;

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(hood.ten())
                .stopAndAdd(index.outtakeIndex1())
                .strafeToLinearHeading(shootVector, shootHeading)
                .stopAndAdd(new SequentialAction(
                        new ShootThreeBalls2(),
                        new InstantAction(() -> postIntake100 = true)
                ))
                .afterTime(0.8, intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(-10.5, -27), Math.toRadians(-90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third + 1, index.intakeIndex3())
                .lineToY(-58)
                .waitSeconds(0.5)
                .stopAndAdd(new SequentialAction(
                        intake.IntakeBallStop(),
                        index.intakeIndex1(),
                        new InstantAction(() -> postIntake100 = false)
                ))
                .build();

        Action postIntake = drive.actionBuilder(new Pose2d(-10.5, -58, Math.toRadians(-90)))
                .strafeToLinearHeading(shootVector, shootHeading)
                .stopAndAdd(new SequentialAction(
                        new ShootThreeBalls2(),
                        new InstantAction(() -> postIntake100 = false)
                ))
                .build();

        Action postIntake2 = drive.actionBuilder(new Pose2d(shootVector, shootHeading))
                .stopAndAdd(intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(15, -27), Math.toRadians(-90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second - 1, index.intakeIndex2())
                .afterDisp(third + offset, index.intakeIndex3())
                .lineToY(-63)
                .waitSeconds(0.85/3)
                .stopAndAdd(new ParallelAction(
                        intake.IntakeBallStop(),
                        index.outtakeIndex1()
                ))
                .build();

        Action postIntake4 = drive.actionBuilder(new Pose2d(15, -61, Math.toRadians(-90)))
                .lineToY(-50)
                .splineToSplineHeading(new Pose2d(shootVector, shootHeading), Math.toRadians(-(270-125)))
                .stopAndAdd(new SequentialAction(
                        new ShootThreeBalls2(),
                        new InstantAction(() -> postIntake100 = false)
                ))
                .build();

        Action postIntake5 = drive.actionBuilder(new Pose2d(shootVector, shootHeading))
                .splineToLinearHeading(new Pose2d(new Vector2d(11.5, -60), Math.toRadians(-127)), Math.toRadians(-127))
                .stopAndAdd(intake.IntakeBallReverse())
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex1())
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex2())
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex3())
                .stopAndAdd(intake.IntakeBallStop())
                .build();

        Actions.runBlocking(new ParallelAction(flywheel1.PIDp1(), closePassive));

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> postIntake100 = true),
                new ParallelAction(flywheel1.PIDp1(), postIntake),
                postIntake2
        ));

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> postIntake100 = true),
                new ParallelAction(flywheel1.PIDp1(), postIntake4)
        ));

        Actions.runBlocking(postIntake5);
    }
}