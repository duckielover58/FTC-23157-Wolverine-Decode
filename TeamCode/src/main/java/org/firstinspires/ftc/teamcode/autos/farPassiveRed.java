package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.LLstart;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearing;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.far;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.lockSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.mainTag;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.maxBearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.postIntake100;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.redTag;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.lang.Math;

@Config
@Autonomous(name = "farPassiveRed", group = "Robot")
public class farPassiveRed extends LinearOpMode {

    private Index index;
    private Intake intake;
    private Push push;
    private Hood hood;
    private CRServo swivel;
    private Flywheel flywheel;
    private Limelight3A limelight;

    double kP = 0.35;
    double i = 0;
    boolean upies = false;
    int first = 5;
    int offset = 5;
    int second = first + offset;
    int third = second + offset;
    boolean servoLocked = false;
    @Override
    public void runOpMode() {

        mainTag = redTag;

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)));

        index = new Index(hardwareMap);
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Actions.runBlocking(index.indexHome());
        Actions.runBlocking(push.PushBallDown());
        Actions.runBlocking(hood.hoodPositionInit());
        Actions.runBlocking(flywheel.flywheelInit());

        while (!servoLocked || !isStarted()) {
            lodk(mainTag);
            telemetry.addData("Locking: ", servoLocked);
            telemetry.update();
        }

        telemetry.addLine("Locked");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Action shootThreeBalls = drive.actionBuilder(new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)))
                .waitSeconds(5)
                .stopAndAdd(new SequentialAction(
                        hood.ten(),
                        index.outtakeIndex1(),
                        push.PushBallUp(),

                        hood.eight(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.1),
                        index.outtakeIndex2(),
                        new SleepAction(0.15),
                        push.PushBallUp(),

                        hood.six(),
                        new SleepAction(0.2),
                        push.PushBallDown(),
                        new SleepAction(0.1),
                        index.outtakeIndex3(),
                        new SleepAction(0.15),
                        push.PushBallUp(),

                        new SleepAction(0.2),
                        push.PushBallDown(),
                        hood.ten(),
                        new SleepAction(0.05), //TODO is this needed? (change to 0.15 if not)
                        index.intakeIndex1()
                ))
                .stopAndAdd(new InstantAction(() -> postIntake100 = false))
                .build();

        Action farPassive = drive.actionBuilder(new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)))
                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(((24*3)-35), 30), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(63)
                .waitSeconds(0.5)
//                .splineToLinearHeading(new Pose2d(((24*3)-35), 63, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallStop())
                .strafeToLinearHeading(new Vector2d(((24*3)-(16.75/2)), 17.5/2), Math.toRadians(90))
                .build();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> postIntake100 = true),
                new ParallelAction(
                    shootThreeBalls,
                    flywheel.PIDp2()
                ),
                farPassive,
                new InstantAction(() -> postIntake100 = true),
                new ParallelAction(
                    shootThreeBalls,
                    flywheel.PIDp2()
                )
        ));
    }

    void lodk (int tag) {
        if (!LLstart) {
            limelight.start();
            LLstart = true;
        }
        telemetry.addLine("Started");
        limelight.pipelineSwitch(tag);
        LLResult result = limelight.getLatestResult();
        telemetry.addLine("Result?");
        if (result != null) {
            if (result.isValid()) {
                telemetry.addLine("In Loop");
                bearing = result.getTx();
                bearingErr = bearing - maxBearingErr;
                lockSpeed = 0.1 * bearingErr;
                lockSpeed = Math.max(-0.17, Math.min((lockSpeed), 0.17));
                if (bearing > 0) {
                    swivel.setDirection(CRServo.Direction.FORWARD);
                    swivel.setPower(-lockSpeed);
                } else if (bearing < -1) {
                    swivel.setDirection(CRServo.Direction.REVERSE);
                    swivel.setPower(lockSpeed);
                } else {
                    swivel.setPower(0);
                    servoLocked = true;
                }
            } else swivel.setPower(0);
        } else swivel.setPower(0);
    }
}