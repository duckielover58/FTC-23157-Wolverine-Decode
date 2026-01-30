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

        Actions.runBlocking(farPassive);
    }
}