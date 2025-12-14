package org.firstinspires.ftc.teamcode.testingFiles;


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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;


@Config
@Autonomous(name = "fakeBlueAuto", group = "Robot")
public class fakeBlueAuto extends LinearOpMode {

    private Flywheel flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Swivel swivel;
    private Limelight limelight;

    public static final int RED_TAG_ID = 24;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBalls() {
            sequence = new SequentialAction(

                    flywheel.shoot(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(2.75),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.index2(),
                    new SleepAction(1.5),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.index3(),
                    new SleepAction(0.5),
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
        Pose2d startPose = new Pose2d(-61, -9, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        flywheel = new Flywheel(hardwareMap);
        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        limelight = new Limelight(hardwareMap);

        waitForStart();

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(new ShootThreeBalls())
                .strafeToLinearHeading(new Vector2d(-9.5,30),Math.toRadians(270-180))
                .afterTime(0.3, intake.IntakeBall())
                .stopAndAdd(index.index1())
                .strafeTo(new Vector2d(-9.5, 33))
                .waitSeconds(.85)
                .stopAndAdd(index.index2())
                .strafeTo(new Vector2d(-9.5, 36))
                .waitSeconds(.85)
                .stopAndAdd(index.index3())
                .strafeTo(new Vector2d(-9.5, 41))
                .waitSeconds(.85)
                .stopAndAdd(intake.IntakeBallStop())
                .strafeToLinearHeading(new Vector2d(-12, 0), Math.toRadians(135-180))
                .stopAndAdd(limelight.limelightRed())
                .stopAndAdd(new ShootThreeBalls())
                .waitSeconds(5)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(11, 31, Math.toRadians(270-180)), Math.toRadians(270-180))
                .afterTime(0.3, intake.IntakeBall())
                .strafeTo(new Vector2d(11, 50))
                .strafeToLinearHeading(new Vector2d(-29.3, 30.3), Math.toRadians(135-180))
                .afterTime(0.3, intake.IntakeBallStop())
//                .stopAndAdd(limelight.limelightRed())
                .waitSeconds(5)
                .stopAndAdd(new ShootThreeBalls())
                .strafeToLinearHeading(new Vector2d(-29.3, 30.3), Math.toRadians(180-180))
                .stopAndAdd(new ShootThreeBalls())
                .build();

        Action fullRoutine = new SequentialAction(closePassive);

        Actions.runBlocking(fullRoutine);
    }
}