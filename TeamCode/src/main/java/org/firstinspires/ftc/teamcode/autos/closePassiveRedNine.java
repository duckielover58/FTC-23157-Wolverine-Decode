package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.close;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearRed.*;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.postIntake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "closePassiveRedNine", group = "Robot")
public class closePassiveRedNine extends LinearOpMode {

    //floor0 - constants
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;
    private Flywheel flywheel1;

    double kP = 0.35;
    double first = 8;
    double offset = 4.5;
    double second = first + offset;
    double third = second + offset;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    hood.ten(),
                    index.outtakeIndex1(),
                    push.PushBallUp(),

                    hood.seven(),
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex2(),
                    new SleepAction(0.15),
                    push.PushBallUp(),

                    hood.five(),
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex3(),
                    new SleepAction(0.15),
                    push.PushBallUp(),

                    new SleepAction(0.1),
                    push.PushBallDown(),
//                    new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    } // 12.75-12.9 voltage


    private class ShootThreeBalls2 implements Action {
        private final Action sequence;

        public ShootThreeBalls2() {
            sequence = new SequentialAction(
                    hood.ten(),
                    index.outtakeIndex1(),
                    push.PushBallUp(),

//                    hood.seven(),
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
                    new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    }

    @Override
    //floor1
    public void runOpMode() {

        Pose2d startPose = new Pose2d(startX, startY , startH);
        Pose2d endShootPose = new Pose2d(-10.5, 50, 90);
        Pose2d endShootPoseTwo = new Pose2d(15, 61, 90);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);
        Vector2d shootVector = new Vector2d(-11, 11);
        double shootHeading = Math.toRadians(125);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel1 = new Flywheel(hardwareMap);

        Actions.runBlocking(index.outtakeIndex1());
        Actions.runBlocking(push.PushBallDown());
        Actions.runBlocking(hood.ten());

        waitForStart();
        if (isStopRequested()) return;

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(hood.ten())
                .stopAndAdd(index.outtakeIndex1())
                .strafeToLinearHeading(shootVector, shootHeading)
              //  .strafeToLinearHeading(new Vector2d(tempX, tempY), tempH)
              //  .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(125))
                .stopAndAdd(new SequentialAction(
                        new ShootThreeBalls2(),
                        new InstantAction(() -> postIntake = true)
                ))
                .afterTime(0.8, intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(-10.5, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third + 1, index.intakeIndex3())
                .lineToY(58)
                .waitSeconds(0.5)
                .stopAndAdd(new SequentialAction(
                        intake.IntakeBallStop(),
                        index.intakeIndex1(),
                        new InstantAction(() -> postIntake = false)
                ))
                .build();

        Action postIntake = drive.actionBuilder(endShootPose)
//                .stopAndAdd(new StartRevShort())
                .strafeToLinearHeading(shootVector, shootHeading)
                .stopAndAdd(new SequentialAction(
                        new ShootThreeBalls2(),
                        new InstantAction(() -> GlobalVariable.postIntake = true)
                ))
                .build();

        Action postIntake2 = drive.actionBuilder(new Pose2d(shootVector, shootHeading))
                .stopAndAdd(intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(15, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second - 1, index.intakeIndex2())
                .afterDisp(third + offset, index.intakeIndex3())
                .lineToY(63)
                .waitSeconds(0.85/3)
//                .stopAndAdd(index.intakeIndex2())
//                .waitSeconds(0.85/3)
//                .stopAndAdd(index.intakeIndex3())
//                .waitSeconds(0.85/3)
//                .stopAndAdd(index.intakeIndex1())
                .stopAndAdd(new ParallelAction(
                        intake.IntakeBallStop(),
                        index.outtakeIndex1()
                ))
                .build();

        Action postIntake4 = drive.actionBuilder(new Pose2d(15, 61, Math.toRadians(90)))
                .lineToY(50)
                .splineToSplineHeading(new Pose2d(shootVector, shootHeading), Math.toRadians(270-125))
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> GlobalVariable.postIntake = false),
                        new ShootThreeBalls2(),
                        new InstantAction(() -> GlobalVariable.postIntake = true)
                ))
                .splineToLinearHeading(new Pose2d(new Vector2d(11.5, 60), Math.toRadians(127)), Math.toRadians(127))
                .stopAndAdd(intake.IntakeBallReverse()) //TODO find good speeds for this
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex1())
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex2())
                .waitSeconds(0.5)
                .stopAndAdd(index.intakeIndex3())
                .stopAndAdd(intake.IntakeBallStop())
                //TODO add last shooting sequence
                .build();

        //TODO adjust the flywheel speed before every shooting cycle depending on power
        Actions.runBlocking(new ParallelAction(
                flywheel1.PIDp1(),
                closePassive));
        Actions.runBlocking(new SequentialAction(new ParallelAction(
                flywheel1.PIDp1(),
                new SequentialAction(
                        postIntake,
                        new InstantAction(() -> GlobalVariable.postIntake = true),
                        new InstantAction(() -> GlobalVariable.postIntake = false)
                        )),
                postIntake2
        ));
        Actions.runBlocking(postIntake4);
    }
}
