package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake1X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.Intake1Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.IntakeH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.PreIntake1X;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.PreIntake1Y;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.PreIntakeH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.shootShortY;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startY;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;

@Config
@Autonomous(name = "jewblue", group = "Robot")
public class jewblue extends LinearOpMode {

    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private Swivel swivel;

    private static final double SHOOT_VELOCITY = 840;

    // -------------------- ACTIONS --------------------

    private class FlywheelPIDAction implements Action {
        private final double target;

        public FlywheelPIDAction(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!opModeIsActive()) {
                flywheel.setPower(0);
                return false;
            }

            double currentVelocity = flywheel.getVelocity();
            double error = target - currentVelocity;
            double ff = kF * target;
            double output = ff + (kP * error);
            output = Math.max(0.0, Math.min(1.0, output));
            flywheel.setPower(output);

            return true;
        }
    }

    private class StopFlywheelAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheel.setPower(0);
            return false;
        }
    }

    private class ShootThreeBalls implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    new SleepAction(6.5),
                    index.intakeIndex2(),
                    new SleepAction(0.2),
                    push.PushBallDown(),
                    new SleepAction(1.3),
                    push.PushBallUp(),
                    hood.seven(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.45),
                    index.intakeIndex3(),
                    new SleepAction(0.25),
                    push.PushBallUp(),
                    hood.six(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    index.intakeIndex1(),
                    new SleepAction(0.3),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    index.intakeIndex1()
            );
        }
    }

    private class IntakeRoutine implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public IntakeRoutine() {
            sequence = new SequentialAction(
                    intake.IntakeBall(),
                    index.intakeTurnRight(),
                    index.intakeTurnRight(),
                    index.intakeTurnRight(),
                    index.intakeTurnRight()
            );
        }
    }

    // -------------------- OPMODE --------------------

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d startPose = new Pose2d(startX, startY, startH);
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
        hood = new Hood(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        Action closePassive = drive.actionBuilder(startPose)
                .stopAndAdd(
                        new ParallelAction(
                                new FlywheelPIDAction(SHOOT_VELOCITY),
                                drive.actionBuilder(startPose)
                                        .splineToLinearHeading(
                                                new Pose2d(shootShortX, shootShortY, shootH),
                                                shootH
                                        )
                                        .stopAndAdd(hood.shortHoodPos())
                                        .stopAndAdd(new ShootThreeBalls())
                                        .build()
                        )
                )
                .stopAndAdd(new StopFlywheelAction())
                .setTangent(-20)
                .splineToLinearHeading(
                        new Pose2d(PreIntake1X, PreIntake1Y, PreIntakeH),
                        Math.toRadians(270)
                )
                .afterTime(0.3, new SequentialAction(
                        intake.IntakeBall(),
                        index.intakeIndex1()
                ))
                .strafeToLinearHeading(
                        new Vector2d(Intake1X, Intake1Y),
                        IntakeH
                )
                .stopAndAdd(new IntakeRoutine())
                .build();

        Actions.runBlocking(closePassive);
    }
}
