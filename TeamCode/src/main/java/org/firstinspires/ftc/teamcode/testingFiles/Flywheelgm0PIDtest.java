package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Disabled
@Config
@Autonomous(name = "Flywheelgm0PIDTest")
public class Flywheelgm0PIDtest extends LinearOpMode {

    public static double kF = 0.00042;
    public static double kP = 0.00009;
    public static double targetVelocity = 740;

    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FtcDashboard Dashboard = FtcDashboard.getInstance();
        telemetry = Dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double currentVelocity = flywheel.getVelocity();
            double error = targetVelocity - currentVelocity;

            double ff = kF * targetVelocity;

            double output = ff + (kP * error);

            output = Math.max(0.0, Math.min(1.0, output));

            flywheel.setPower(output);

            telemetry.addData("Target Velocity: ", targetVelocity);
            telemetry.addData("Current Velocity: ", currentVelocity);
            telemetry.addData("Erorr: ", error);
            telemetry.addData("Power: ", output);
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", currentVelocity);
            packet.put("Error", error);
            packet.put("Power", output);
            telemetry.update();
        }
    }
}