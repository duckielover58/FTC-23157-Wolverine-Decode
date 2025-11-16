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
@Autonomous(name = "closePassive", group = "Robot")
public class closePassive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-61, -9, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        waitForStart();

        Action farPassive = drive.actionBuilder(startPose)
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(-12.5,-31),Math.toRadians(270))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12.5, -36))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12.5, -41))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(225))
                .waitSeconds(4.5)
                .setTangent(45)
                .splineToLinearHeading(new Pose2d(11, -31, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(1)
                .strafeTo(new Vector2d(11, -36))
                .waitSeconds(1)
                .strafeTo(new Vector2d(11, -41))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(225))
                .waitSeconds(4.5)
                .strafeToLinearHeading(new Vector2d(-29.3, -30.3), Math.toRadians(180))
                .build();

        Action fullRoutine = new SequentialAction(farPassive);

        Actions.runBlocking(fullRoutine);
    }
}
