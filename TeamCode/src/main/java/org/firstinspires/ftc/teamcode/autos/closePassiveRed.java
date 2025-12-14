package org.firstinspires.ftc.teamcode.autos;


// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;


@Config
@Autonomous(name = "closePassiveRed", group = "Robot")
public class closePassiveRed extends LinearOpMode {
    //

    private Flywheel flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Swivel swivel;
    private Limelight3A limelight;
    int colorPipeline = 3;

    //GGP 21
    //PGP 22
    //PPG 23


    public static final int RED_TAG_ID = 24;

    private class ShootThreeBallsPPG implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsPPG() {
            sequence = new SequentialAction(
                    index.index2(),
                    flywheel.shoot(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(2.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index3(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index1(),
                    new SleepAction(0.65),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsPGP implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsPGP() {
            sequence = new SequentialAction(
                    index.index2(),
                    flywheel.shoot(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(2.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index1(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index3(),
                    new SleepAction(0.65),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsGPP implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsGPP() {
            sequence = new SequentialAction(
                    index.index1(),
                    flywheel.shoot(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(2.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index2(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index3(),
                    new SleepAction(0.65),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsCornerPPG implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsCornerPPG() {
            sequence = new SequentialAction(
                    index.index1(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index2(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index3(),
                    new SleepAction(0.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsCornerPGP implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsCornerPGP() {
            sequence = new SequentialAction(
                    index.index1(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index3(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index2(),
                    new SleepAction(0.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsCornerGPP implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsCornerGPP() {
            sequence = new SequentialAction(
                    index.index3(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index2(),
                    new SleepAction(0.85),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),


                    index.index1(),
                    new SleepAction(0.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    flywheel.shootStop(),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-49, -49, Math.toRadians(55));
        Pose2d shootPose = new Pose2d(-12, 0, Math.toRadians(135));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        flywheel = new Flywheel(hardwareMap);
        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();

        limelight.start();
        limelight.pipelineSwitch(colorPipeline);
        LLResult results = limelight.getLatestResult();
        sleep(100);

        if (!results.isValid()) {
            colorPipeline = 4;
            limelight.pipelineSwitch(colorPipeline);
            results = limelight.getLatestResult();
            sleep(100);
            if (!results.isValid()) {
                colorPipeline = 5;
            }
        }

        Action closePassivetab1PPG = drive.actionBuilder(startPose)
                .stopAndAdd(new ShootThreeBallsPPG())
                .build();

        Action closePassivetab1PGP = drive.actionBuilder(startPose)
                .stopAndAdd(new ShootThreeBallsPGP())
                .build();

        Action closePassivetab1GPP = drive.actionBuilder(startPose)
                .stopAndAdd(new ShootThreeBallsGPP())
                .build();

        Action closePassivetab2 = drive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-9.5,-30, Math.toRadians(270)),Math.toRadians(270))
                .afterTime(0.3, intake.IntakeBallReverse())
                .stopAndAdd(index.index1())
                .strafeTo(new Vector2d(-9.5, -34.5))
                .stopAndAdd(index.index2())
                .strafeTo(new Vector2d(-9.5, -39))
                .waitSeconds(0.85)
                .stopAndAdd(index.index3())
                .strafeTo(new Vector2d(-9.5, -43.5))
                .waitSeconds(0.85)
                .stopAndAdd(intake.IntakeBallStop())
                .stopAndAdd(flywheel.shoot())
                .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(135))
                .build();

        Action closePassivetab3PPG = drive.actionBuilder(shootPose)
                .stopAndAdd(new ShootThreeBallsCornerPPG())
                .build();
        Action closePassivetab3PGP = drive.actionBuilder(shootPose)
                .stopAndAdd(new ShootThreeBallsCornerPGP())
                .build();
        Action closePassivetab3GPP = drive.actionBuilder(shootPose)
                .stopAndAdd(new ShootThreeBallsCornerGPP())
                .build();

        Action closePassivetab4 = drive.actionBuilder(shootPose)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(14, -30, Math.toRadians(270)), Math.toRadians(270))
                .stopAndAdd(intake.IntakeBallReverse())
                .stopAndAdd(index.index1())
                .strafeTo(new Vector2d(14, -35))
                .waitSeconds(0.85)
                .stopAndAdd(index.index2())
                .strafeTo(new Vector2d(14, -39))
                .waitSeconds(0.85)
                .stopAndAdd(index.index3())
                .strafeTo(new Vector2d(14, -43.5))
                .waitSeconds(0.85)
                .stopAndAdd(intake.IntakeBallStop())
                .build();

        if (colorPipeline == 3) {
            Actions.runBlocking(
                    new SequentialAction(
                        closePassivetab1PPG,
                        closePassivetab2,
                        closePassivetab3GPP,
                        closePassivetab4
                    )
            );
        } else if (colorPipeline == 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            closePassivetab1PPG,
                            closePassivetab2,
                            closePassivetab3GPP,
                            closePassivetab4
                    )
            );
        } else if (colorPipeline == 5) {
            Actions.runBlocking(
                    new SequentialAction(
                            closePassivetab1PPG,
                            closePassivetab2,
                            closePassivetab3GPP,
                            closePassivetab4
                    )
            );
        }

    }
}
