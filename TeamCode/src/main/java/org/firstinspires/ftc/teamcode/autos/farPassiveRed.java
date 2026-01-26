package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.far;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "farPassiveRed", group = "Robot")
public class farPassiveRed extends LinearOpMode {

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

    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }
    boolean shootFar() {
        while (i < 4) {
            sleep(20);
            telemetry.addLine("In loop");
            telemetry.addData("i: ", i);
            flywheelPID(far);
            if (flywheel.getVelocity() >= far * 0.9 && flywheel.getVelocity() <= far * 1.1) {
                telemetry.addLine("running shot");
                i++;
                upies = true;

                if (i == 1) {
                    Actions.runBlocking(new SequentialAction(index.outtakeIndex1()));
                } else if (i == 2) {
                    Actions.runBlocking(new SequentialAction(index.outtakeIndex2()));
                } else {
                    Actions.runBlocking(new SequentialAction(index.outtakeIndex3()));
                }

                Actions.runBlocking(new SequentialAction(
                        new SleepAction(200),
                        push.PushBallUp()
                ));
            } else {
                if (upies) {
                    sleep(200);
                    Actions.runBlocking(push.PushBallDown());
                    upies = false;
                }
            }
            telemetry.update();
        }
        telemetry.addLine("returning false");
        telemetry.update();
        return false;
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

        waitForStart();
        if (isStopRequested()) return;


        Action farPassive = drive.actionBuilder(new Pose2d(((24*3)-(16.75/2)), 17.5/2, Math.toRadians(90)))

                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(((24*3)-35), 30), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallReverse())
                .afterDisp(first, index.intakeIndex1())
                .afterDisp(second, index.intakeIndex2())
                .afterDisp(third, index.intakeIndex3())
                .lineToY(63)
                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(((24*3)-35), 63, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(intake.IntakeBallStop())
                .strafeToLinearHeading(new Vector2d(((24*3)-(16.75/2)), 17.5/2), Math.toRadians(90))
                .build();
/*
        while (true) {
            shootFar();
            if (!shootFar()) {
                break;
            }
        }

 */
        Actions.runBlocking(farPassive);
        while (true) {
            shootFar();
            if (!shootFar()) {
                break;
            }
        }
    }
}