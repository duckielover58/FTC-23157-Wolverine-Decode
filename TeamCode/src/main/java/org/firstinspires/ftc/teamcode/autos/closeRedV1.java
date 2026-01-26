package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearRed.*;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.targetVelocity;

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
@Autonomous(name = "closeRedV1", group = "Robot")
public class closeRedV1 extends LinearOpMode {

    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;

    double kP = 0.35;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    new InstantAction(() -> flywheel.setPower(1)),
                    new SleepAction(0.4),
                    hood.ten(),
                    index.outtakeIndex1(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.7),
                    push.PushBallUp(),

                    hood.ten(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.45),
                    index.outtakeIndex2(),
                    new SleepAction(0.40),
                    push.PushBallUp(),

                    hood.ten(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.outtakeIndex3(),
                    new SleepAction(1),
                    push.PushBallUp(),

                    new SleepAction(0.3),
                    new InstantAction(() -> flywheelPID(0)),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.intakeIndex1()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    }

    private class StartRevShort implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheelPID(2000);
            return false;
        }
    }

    private class ShootThreeBallsCorner implements Action {
        private final Action sequence;

        public ShootThreeBallsCorner() {
            sequence = new SequentialAction(
                    index.outtakeIndex3(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(1.9),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.3),
                    index.outtakeIndex2()
            );
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
    }

    private class ShootThreeBallsCornerTwo implements Action {
        private final Action sequence;

        public ShootThreeBallsCornerTwo() {
            sequence = new SequentialAction(
                    new StartRevShort(),
                    new SleepAction(1.0),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.intakeIndex1(),
                    new SleepAction(0.6),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    new InstantAction(() -> flywheelPID(0)),
                    new SleepAction(0.2),
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
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel2");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d startPose = new Pose2d(startX, startY , startH);
        Pose2d endShootPose = new Pose2d(-10.5, 50, 90);

        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);

        Actions.runBlocking(hood.hoodPositionInit());

        waitForStart();
        if (isStopRequested()) return;

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(index.intakeIndex1())
                .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(125))
                .stopAndAdd(new ShootThreeBalls())
                .afterTime(0.8, intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(-10.5, 23), Math.toRadians(90))
                .waitSeconds(0.2)
                .stopAndAdd(index.intakeIndex1())
                .strafeTo(new Vector2d(-10.5, 52))
                .waitSeconds(0.45)
                .stopAndAdd(index.intakeIndex2())
                .waitSeconds(0.9)
                .stopAndAdd(index.intakeIndex3())
                .waitSeconds(0.8)
                .stopAndAdd(intake.IntakeBallStop())
                .build();

        Action postIntake = drive.actionBuilder(endShootPose)
                .stopAndAdd(new StartRevShort())
                .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(125))
                .build();

        Action postIntake2 = drive.actionBuilder(
                        new Pose2d(new Vector2d(-12, 0), Math.toRadians(120)))
                .stopAndAdd(new ShootThreeBalls())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(15, 23, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .waitSeconds(0.2)
                .stopAndAdd(index.intakeIndex1())
                .strafeTo(new Vector2d(15, 58))
                .waitSeconds(0.45)
                .stopAndAdd(index.intakeIndex2())
                .waitSeconds(0.9)
                .stopAndAdd(index.intakeIndex3())
                .waitSeconds(0.8)
                .stopAndAdd(intake.IntakeBallStop())
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
    }
}
