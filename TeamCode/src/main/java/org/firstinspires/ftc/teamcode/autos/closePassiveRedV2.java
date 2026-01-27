package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearRed.*;

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

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "closePassiveRedV2", group = "Robot")
public class closePassiveRedV2 extends LinearOpMode {

    //floor0 - constants
    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;

    double kP = 0.35;
    int first = 8;
    int offset = 5;
    int second = first + offset;
    int third = second + offset;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    index.outtakeIndex1(),
                    hood.three(),
                    new InstantAction(() -> flywheel.setPower(1)),
                    new SleepAction(0.7),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    push.PushBallUp(),

                    hood.three(),
                    new SleepAction(0.23),
                    push.PushBallDown(),
                    new SleepAction(0.2),
                    index.outtakeIndex2(),
                    new SleepAction(0.2),
                    push.PushBallUp(),

                    hood.three(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.2),
                    index.outtakeIndex3(),
                    new SleepAction(0.25),
                    push.PushBallUp(),

                    new SleepAction(0.3),
                    new InstantAction(() -> flywheelPID(0)),
                    push.PushBallDown(),
                    new SleepAction(0.01), //TODO is this needed? (change to 0.15 if not)
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    } // 12.75-12.9 voltage


    private class StartRevShort implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheelPID(2000);
            return false;
        }
    }

    private class SecondShootThreeBalls implements Action {
        private final Action sequence;

        public SecondShootThreeBalls() { // 12.7-12.9 voltage
            sequence = new SequentialAction(
                    hood.three(),
                    new InstantAction(() -> flywheel.setPower(1)),
                    new SleepAction(0.23),
                    push.PushBallDown(),
                    push.PushBallUp(),

                    hood.three(),
                    new SleepAction(0.25),
                    push.PushBallDown(),
                    new SleepAction(0.15),
                    index.outtakeIndex2(),
                    new SleepAction(0.25),
                    push.PushBallUp(),

                    hood.three(),
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new SleepAction(0.15),
                    index.outtakeIndex3(),
                    new SleepAction(0.15),
                    push.PushBallUp(),

                    new SleepAction(0.3),
                    new InstantAction(() -> flywheelPID(0)),
                    push.PushBallDown(),
                    new SleepAction(0.01), //TODO if this is just to switch intake, this time can be 0.15
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    }

    private class ThirdShootThreeBalls implements Action {
        private final Action sequence;

        public ThirdShootThreeBalls() { // -16, 16 - test if this location works for third shooting - can change hood angle with it
            sequence = new SequentialAction(
                    hood.four(),
                    new InstantAction(() -> flywheel.setPower(1)),
//                    new SleepAction(0.1),
                    push.PushBallDown(),
                    new SleepAction(0.25),
                    push.PushBallUp(),

                    hood.four(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.3),
                    index.outtakeIndex3(),
                    new SleepAction(0.3),
                    push.PushBallUp(),

                    hood.four(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.25),
                    index.outtakeIndex1(),
                    new SleepAction(0.2),
                    push.PushBallUp(),

                    new SleepAction(0.2),
                    new InstantAction(() -> flywheelPID(0)),
                    push.PushBallDown(),
                    new SleepAction(0.01), //TODO if this is just to switch intake, this time can be 0.15
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
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

    @Override
    //floor1
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel2");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d startPose = new Pose2d(startX, startY , startH);
        Pose2d endShootPose = new Pose2d(-10.5, 50, 90);
        Pose2d endShootPoseTwo = new Pose2d(15, 61, 90);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);

        Actions.runBlocking(index.indexHome());
        Actions.runBlocking(push.PushBallDown());

        waitForStart();
        if (isStopRequested()) return;

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(index.outtakeIndex1())
                .stopAndAdd(hood.three())
                .strafeToLinearHeading(new Vector2d(-40, 32), Math.toRadians(125))
                //  .strafeToLinearHeading(new Vector2d(tempX, tempY), tempH)
                //  .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(125))
                .stopAndAdd(new ShootThreeBalls())
                .afterTime(0.8, intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(-10.5, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(57)
                .waitSeconds(0.5)
                .stopAndAdd(intake.IntakeBallStop())
                .waitSeconds(0.2)
                .stopAndAdd(index.outtakeIndex1())
                .build();

        Action postIntake = drive.actionBuilder(endShootPose)
                .stopAndAdd(new StartRevShort())
                .strafeToLinearHeading(new Vector2d(-40, 32), Math.toRadians(125))
                .build();

        Action postIntake2 = drive.actionBuilder(
                        new Pose2d(new Vector2d(-40, 32), Math.toRadians(125)))
                .stopAndAdd(new SecondShootThreeBalls())
                .stopAndAdd(intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(15, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(58)
                .waitSeconds(0.85)
                .stopAndAdd(intake.IntakeBallStop())
                .waitSeconds(0.1)
                .stopAndAdd(index.outtakeIndex2())
                .build();
//
        Action postIntake3 = drive.actionBuilder(endShootPoseTwo)
                .strafeToLinearHeading(new Vector2d(15, 50), Math.toRadians(90))
                .stopAndAdd(new StartRevShort())
                .build();

        Action postIntake4 = drive.actionBuilder(
                new Pose2d(new Vector2d(15, 50), Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-16, 16, Math.toRadians(133)), Math.toRadians(180))
                .waitSeconds(0.45)
                .stopAndAdd(new ThirdShootThreeBalls())
                .stopAndAdd(push.PushBallDown())
                .strafeToLinearHeading(new Vector2d(16.5,53.5),Math.toRadians(127))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        hood.hoodPositionInit(),
                        hood.hoodUp(),
                        hood.hoodUp(),
                        hood.hoodUp()
                )
        );

        flywheel.setPower(1);
        Actions.runBlocking(closePassive);
        flywheel.setPower(1);
        Actions.runBlocking(postIntake);
        flywheel.setPower(1);
        Actions.runBlocking(new SequentialAction(hood.hoodDown(), hood.hoodDown()));
        Actions.runBlocking(postIntake2);
        Actions.runBlocking(postIntake3);
        Actions.runBlocking(postIntake4);
    }
}
