package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;


@Config
@Autonomous(name = "farPassive", group = "Robot")
public class farPassiveBlue extends LinearOpMode {
    private DcMotorEx flywheel;
    private Index index;
    private Intake intake;
    private Push push;
    private Swivel swivel;
//    private Limelight3A limelight;
    private Hood hood;
    int colorPipeline = 3;
    boolean servoLocked = true;
    double ServoPower = 1.0;
    public final int redTag = 0;
    double previousTime = 0;
    double previousError = 0;
    double integralSum = 0;
    boolean rumble = false;
    double kP = 0.35;
    double kI = 0.000001;
    double kD = 0.5;


    private class ShootThreeBalls implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBalls() {
            sequence = new SequentialAction(
                    index.index2(),
                    new InstantAction(() -> flywheelPID(840)),
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
                    new SleepAction(1.0),
                    push.PushBallUp(),
                    new SleepAction(0.3),
                    push.PushBallDown(),
                    new SleepAction(0.5),
                    new InstantAction(() -> flywheelPID(0)),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }
    private class ShootThreeBallsCorner implements Action {
        private final Action sequence;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return sequence.run(packet);
        }

        public ShootThreeBallsCorner() {
            sequence = new SequentialAction(
                    new InstantAction(() -> flywheelPID(750)),
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
                    new InstantAction(() -> flywheelPID(0)),
                    new SleepAction(0.2),
                    index.index1()
            );
        }
    }

    void flywheelPID (double target) {
        previousTime = getRuntime();

        double targetVelocity = target;
        double currentVelocity = flywheel.getVelocity();
        double currentTime = getRuntime();
        double dt = currentTime - previousTime;
        double error = targetVelocity - currentVelocity;

        integralSum += error * dt;
        double derivative = (error - previousError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Math.max(-1.0, Math.min(1.0, output));

        flywheel.setPower(output);

        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Current Velocity: ", currentVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Power: ", output);

        previousError = error;
        previousTime = currentTime;

        if (error <= 40) {
            rumble = true;
        } else {
            rumble = false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Pose2d startPose = new Pose2d(61.5, -21, Math.toRadians(180));
        Pose2d shootPose = new Pose2d(-12, 0, Math.toRadians(135));
        Pose2d endShootPose = new Pose2d(-9.5, -41.5, Math.toRadians(230));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        swivel = new Swivel(hardwareMap);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hood = new Hood(hardwareMap);


        waitForStart();
//840
        Action farPassive = drive.actionBuilder(startPose)
                .stopAndAdd(hood.hoodPosition())
                .stopAndAdd(hood.hoodUp())
                .strafeToLinearHeading(new Vector2d(59,-21),Math.toRadians(200))
                .stopAndAdd(new ShootThreeBalls())
                .strafeToLinearHeading(new Vector2d(34, -28), Math.toRadians(270))
                .afterTime(0.3, intake.IntakeBall())
                .stopAndAdd(index.index1())
                .strafeTo(new Vector2d(-9.5, -32.5))
                .stopAndAdd(index.index2())
                .strafeTo(new Vector2d(-9.5, -37))
                .waitSeconds(0.85)
                .stopAndAdd(index.index3())
                .strafeTo(new Vector2d(-9.5, -41.5))
                .waitSeconds(0.85)
                .stopAndAdd(intake.IntakeBallStop())
                .build();
        Action postIntake = drive.actionBuilder(endShootPose)
                .strafeToLinearHeading(new Vector2d(59,-21),Math.toRadians(200))
                .stopAndAdd(new ShootThreeBallsCorner())
                .strafeToLinearHeading(new Vector2d(11, -28),Math.toRadians(270))
                .afterTime(0.3, intake.IntakeBall())
                .stopAndAdd(index.index1())
                .strafeTo(new Vector2d(-9.5, -32.5))
                .stopAndAdd(index.index2())
                .strafeTo(new Vector2d(-9.5, -37))
                .waitSeconds(0.85)
                .stopAndAdd(index.index3())
                .strafeTo(new Vector2d(-9.5, -41.5))
                .waitSeconds(0.85)
                .stopAndAdd(intake.IntakeBallStop())
                .build();

        Action fullRoutine = new SequentialAction(farPassive, postIntake);
        Actions.runBlocking(fullRoutine);
    }
}