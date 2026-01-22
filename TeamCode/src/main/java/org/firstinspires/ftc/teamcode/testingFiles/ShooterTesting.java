package org.firstinspires.ftc.teamcode.testingFiles;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.*;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kF;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.subsystems.GlobalVariable;
import org.intellij.lang.annotations.JdkConstants;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "ShooterTesting", group = "Robot")
public class ShooterTesting extends LinearOpMode {
    private Limelight3A limelight;
    private Gamepad cG2;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private CRServo swivel;
    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();
    TelemetryPacket packet = new TelemetryPacket();
    Hood hood = new Hood(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException {

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        cG2.copy(gamepad2);
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2 = hardwareMap.get(DcMotorEx.class, "Flywheel2");
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (cG2.right_trigger >= 0.1) {
            lodk(mainTag);
            flywheelPID(close);
            velHoodPos = (flywheel.getVelocity() * (targetHoodClose/close)); // -0.1 * result.getDistance();
            telemetry.addData("Vel Hood Pos: ", velHoodPos);
            telemetry.addData("Velocity: ", flywheel.getVelocity());
            runningActions.add(new SequentialAction(hood.setHoodPosShoot()));
        } else if (cG2.left_trigger >= 0.1) {
            lodk(mainTag);
            flywheelPID(GlobalVariable.far);
            velHoodPos = flywheel.getVelocity() * (targetHoodFar/close);
            telemetry.addData("Vel Hood Pos: ", velHoodPos);
            runningActions.add(new SequentialAction(hood.setHoodPosShoot()));
        } else {
            if (LLstart) {
                limelight.stop();
                LLstart = false;
            }
        }
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
                lockSpeed = Math.max(-0.3, Math.min((lockSpeed), 0.3));
                if (bearing > maxBearingErr) {
                    swivel.setDirection(CRServo.Direction.FORWARD);
                    swivel.setPower(-lockSpeed);
                } else if (bearing < -maxBearingErr) {
                    swivel.setDirection(CRServo.Direction.REVERSE);
                    swivel.setPower(lockSpeed);
                } else {
                    swivel.setPower(0);
                }
            } else swivel.setPower(0);
        } else swivel.setPower(0);
    }

    void flywheelPID (double target) {
        double currentVelocity = flywheel.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);

        flywheelPID2(target);
    }

    void flywheelPID2 (double target) {

        double currentVelocity = flywheel2.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel2.setPower(output);
    }
}

