package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@TeleOp(name = "TestTeleOpv2")
public class TestTeleOpv2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        Intake intake = new Intake(hardwareMap);
        Push push = new Push(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Swivel swivel = new Swivel(hardwareMap);
        Index index = new Index(hardwareMap);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));

            drive.updatePoseEstimate();

            if (gamepad1.right_bumper) {
                Actions.runBlocking(intake.IntakeBall());
            }
            if (gamepad1.left_bumper) {
                Actions.runBlocking(intake.IntakeBallStop());
            }
            if (gamepad2.dpad_up) {
                Actions.runBlocking(push.PushBallUp());
            }
            if (gamepad2.dpad_down) {
                Actions.runBlocking(push.PushBallDown());
            }
            if (gamepad2.right_bumper) {
                Actions.runBlocking(flywheel.shoot());
            }
            if (gamepad2.left_bumper) {
                Actions.runBlocking(flywheel.shootStop());
            }
            if (gamepad2.x) {
                Actions.runBlocking(index.index1());
            }
            if (gamepad2.y) {
                Actions.runBlocking(index.index2());
            }
            if (gamepad2.b) {
                Actions.runBlocking(index.index3());
            }
            /*
            if (gamepad1.dpad_right) {
                Actions.runBlocking(swivel.aim());
            }

             */


            telemetry.update();
        }
    }
}

