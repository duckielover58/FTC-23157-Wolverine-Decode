package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.far;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;

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
@Autonomous(name = "farPassiveRedNoPIDF", group = "Robot")
public class farPassiveRedNoPIDF extends LinearOpMode {

    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;

    double kP = 0.35;
    double i = 0;
    boolean upies = false;
    int first = 5;
    int offset = 5;
    int second = first + offset;
    int third = second + offset;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        public ShootThreeBalls() {
            sequence = new SequentialAction(

                    //floor 1 - outtake1
                    new InstantAction(() -> flywheel.setPower(1)),
                    push.PushBallDown(),
                    index.outtakeIndex1(),
                    hood.seven(), //TODO six or seven not sure
                    new SleepAction(5), //TODO test the time
                    push.PushBallUp(),

                    //floor 2 - outtake2
                    hood.seven(), //TODO six or seven not sure
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    index.outtakeIndex2(),
                    new SleepAction(2.85), //TODO test the time
                    push.PushBallUp(),

                    //floor 3 - outtake3
                    hood.seven(), //TODO six or seven not sure
                    new SleepAction(0.15),
                    push.PushBallDown(),
                    index.outtakeIndex3(),
                    new SleepAction(2.85), //TODO test the time
                    push.PushBallUp(),

                    new SleepAction(0.15),
                    push.PushBallDown(),
                    new InstantAction(() -> flywheel.setPower(0)),
                    new SleepAction(0.15),
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

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel2");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)));

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);

        Actions.runBlocking(index.indexHome());
        Actions.runBlocking(push.PushBallDown());

        Actions.runBlocking(hood.hoodPositionInit());

        waitForStart();
        if (isStopRequested()) return;


        Action farPassive = drive.actionBuilder(new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)))

                //Shots fired
                .stopAndAdd(new ShootThreeBalls())

                //Align properly with the spike mark
                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(((24*3)-35), 30), Math.toRadians(90))

                //Prep the index spinning for slam intake
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())

                //Intake
                .lineToY(63)

                //In case we miss a ball, hopefully this grabs it
                .waitSeconds(0.5)
                .stopAndAdd(intake.IntakeBallStop())

                //Back to og spot
                .strafeToLinearHeading(new Vector2d(((24*3)-(16.75/2)), 17.5/2), Math.toRadians(90))

                //Shots fired
                .stopAndAdd(new ShootThreeBalls())
                .build();

        Actions.runBlocking(farPassive);

    }
}