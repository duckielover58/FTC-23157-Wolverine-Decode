package org.firstinspires.ftc.teamcode.driveClasses;

import android.graphics.HardwareRenderer;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestingActions extends LinearOpMode {
    Intake intake;
    Push push;
    Flywheel flywheel;
    Swivel swivel;
    Index index;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        swivel = new Swivel(hardwareMap);
        index = new Index(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Actions.runBlocking(intake.IntakeBall());
            }
            if (gamepad1.b) {
                Actions.runBlocking(push.PushBallUp());
            }
            if (gamepad1.dpad_down) {
                Actions.runBlocking(push.PushBallDown());
            }
            if (gamepad1.x) {
                Actions.runBlocking(flywheel.shoot());
            }
            if (gamepad1.x) {
                Actions.runBlocking(index.index1());
            }
            if (gamepad1.x) {
                Actions.runBlocking(index.index2());
            }
            if (gamepad1.x) {
                Actions.runBlocking(index.index1());
            }
            if (gamepad1.y) {
                Actions.runBlocking(swivel.aim());
            }
        }
    }
}

