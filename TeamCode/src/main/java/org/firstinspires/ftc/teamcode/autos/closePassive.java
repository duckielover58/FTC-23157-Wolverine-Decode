package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;


@Config
@Autonomous(name = "closePassive", group = "Robot")
public class closePassive extends LinearOpMode {

    private Flywheel flywheel;
    private Index index;
    private Camera camera;
    private Intake intake;
    private Push push;
    private Swivel swivel;

    public static final int RED_TAG_ID = 24;

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }
        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    index.index1(),
                    new ParallelAction(push.PushBallUp(), flywheel.shoot()),
                    push.PushBallDown(),

                    index.index2(),
                    new ParallelAction(push.PushBallUp(), flywheel.shoot()),
                    push.PushBallDown(),

                    index.index3(),
                    new ParallelAction(push.PushBallUp(), flywheel.shoot()),
                    push.PushBallDown()
            );
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-61, -9, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        flywheel = new Flywheel(hardwareMap);
        camera = new Camera(hardwareMap, telemetry);
        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);

        waitForStart();

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(new ShootThreeBalls())
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(-12.5,-31),Math.toRadians(270))
                .afterTime(0.3, intake.IntakeBall())
                .strafeTo(new Vector2d(-12.5, -50))
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(110))
                .afterTime(0.3, intake.IntakeBallStop())
                .stopAndAdd(camera.getCamLock(RED_TAG_ID))
                .stopAndAdd(new ShootThreeBalls())
                .waitSeconds(5)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(11, -31, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0.3, intake.IntakeBall())
                .strafeTo(new Vector2d(11, -50))
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(110))
                .afterTime(0.3, intake.IntakeBallStop())
                .stopAndAdd(camera.getCamLock(RED_TAG_ID))
                .stopAndAdd(new ShootThreeBalls())
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(180))
                .build();

        Action fullRoutine = new SequentialAction(closePassive);

        Actions.runBlocking(fullRoutine);
    }
}
