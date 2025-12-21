package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startH;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startX;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue.startY;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.previousTime;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kF;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.targetVelocity;

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

import org.firstinspires.ftc.teamcode.subsystems.GlobalVariable;
import org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.nearBlue;

import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

// Non-RR imports
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.driveClasses.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

public class nearSideBlue extends LinearOpMode{

    double integralSum = GlobalVariable.integralSum;
    double previousError = GlobalVariable.previousError;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startingPose = new Pose2d(startX, startY, startH);

        waitForStart();
    }

    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = targetVelocity - currentVelocity;

        double ff = kF * targetVelocity;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }
}

