package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "Flywheelgm0PIDTest")
public class Flywheelgm0PIDtest extends LinearOpMode {

    // PID constants (START SMALL)
    public static double kP = 0.05819;
    public static double kI = 0.00002;
    public static double kD = 0.010035;

    public static double integralSum = 0;
    public static double previousError = 0;
    public static double previousTime = 0;
    public static double targetVelocity = 740;

    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FtcDashboard Dashboard = FtcDashboard.getInstance();
        telemetry = Dashboard.getTelemetry();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        previousTime = getRuntime();

        while (opModeIsActive()) {


            double currentVelocity = flywheel.getVelocity();
            double currentTime = getRuntime();
            double dt = currentTime - previousTime;
            double error = targetVelocity - currentVelocity;

            integralSum += error * dt;
            double derivative = (error - previousError) / dt;

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            output = Math.max(-1.0, Math.min(1.0, output));

            flywheel.setPower(output);

            telemetry.addData("Target Velocity: ", targetVelocity);
            telemetry.addData("Current Velocity: ", currentVelocity);
            telemetry.addData("Error: ", error);
            telemetry.addData("Power: ", output);
            telemetry.update();

            previousError = error;
            previousTime = currentTime;
        }
    }
}
