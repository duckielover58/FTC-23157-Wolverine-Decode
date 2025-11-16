package org.firstinspires.ftc.teamcode.autos;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;


@Config
@Autonomous(name = "farPassive", group = "Robot")
public class farPassive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(61.5, -21, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        waitForStart();

        Action farPassive = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(59,-21),Math.toRadians(200))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(34, -31), Math.toRadians(270))
                .waitSeconds(1)
                .lineToY(-36)
                .waitSeconds(1)
                .lineToY(-41)
                .waitSeconds(1)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(59,-21,Math.toRadians(200)),Math.toRadians(90))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(11, -31),Math.toRadians(270))
                .waitSeconds(1)
                .lineToY(-36)
                .waitSeconds(1)
                .lineToY(-41)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-20,-21),Math.toRadians(225))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(-21, -21), Math.toRadians(180))
                .build();

        Action fullRoutine = new SequentialAction(farPassive);

        Actions.runBlocking(fullRoutine);
    }
}