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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "closePassiveRedv3", group = "Robot")
public class closePassiveRedv3 extends LinearOpMode {

    //floor0 - constants
    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;

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
                    new InstantAction(() -> flywheel.setPower(0.3)),
                    push.PushBallUp(),

                    new InstantAction(() -> flywheel.setPower(0.6)),
                    hood.seven(),
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex2(),
                    new SleepAction(0.15),
                    push.PushBallUp(),

                    new InstantAction(() -> flywheel.setPower(0.8)),
                    hood.five(),
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new SleepAction(0.1),
                    index.outtakeIndex3(),
                    new SleepAction(0.15),
                    push.PushBallUp(),

                    new SleepAction(0.1),
                    new InstantAction(() -> flywheelPID(0)),
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
        Vector2d shootVector = new Vector2d(-35, 28);
        double shootHeading = Math.toRadians(125);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);

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
                .stopAndAdd(new ShootThreeBalls())
                .afterTime(0.8, intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(-10.5, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second + 1, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(58)
                .waitSeconds(0.5)
                .stopAndAdd(intake.IntakeBallStop())
                .stopAndAdd(index.outtakeIndex1())
                .build();

        Action postIntake = drive.actionBuilder(endShootPose)
//                .stopAndAdd(new StartRevShort())
                .strafeToLinearHeading(shootVector, shootHeading)
                .stopAndAdd(new ShootThreeBalls())
                .build();

        Action postIntake2 = drive.actionBuilder(new Pose2d(shootVector, shootHeading))
                .stopAndAdd(intake.IntakeBallReverse())
                .strafeToLinearHeading(new Vector2d(15, 27), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third + offset, index.intakeIndex3())
                .lineToY(61)
                .waitSeconds(0.85/3)
                .stopAndAdd(index.intakeIndex2())
                .waitSeconds(0.85/3)
                .stopAndAdd(index.intakeIndex1())
                .waitSeconds(0.85/3)
                .stopAndAdd(index.intakeIndex3())
                .stopAndAdd(new ParallelAction(
                        intake.IntakeBallStop(),
                        index.outtakeIndex1()
                ))
                .build();

        Action postIntake4 = drive.actionBuilder(new Pose2d(15, 61, Math.toRadians(90)))
                .lineToY(50)
                .splineToSplineHeading(new Pose2d(shootVector, shootHeading), Math.toRadians(270-125))
                .stopAndAdd(new ShootThreeBalls())
                .splineToLinearHeading(new Pose2d(new Vector2d(12.5, 60), Math.toRadians(127)), Math.toRadians(127))
                .stopAndAdd(new InstantAction(() -> flywheel.setPower(0)))
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
        flywheel.setPower(0.95);
        Actions.runBlocking(closePassive);
        flywheel.setPower(0.8);
        Actions.runBlocking(postIntake);
        flywheel.setPower(0);
        Actions.runBlocking(postIntake2);
        flywheel.setPower(0.8);
        Actions.runBlocking(postIntake4);
    }
}
