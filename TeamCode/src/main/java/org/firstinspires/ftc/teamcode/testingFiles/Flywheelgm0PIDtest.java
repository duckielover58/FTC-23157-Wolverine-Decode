package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Flywheelgm0PIDTest")
public class Flywheelgm0PIDtest extends LinearOpMode {

    // PID constants (START SMALL)
    double kP = 0.0200;
    double kI = 0.0;
    double kD = 0.00003;

    double integralSum = 0;
    double previousError = 0;
    double previousTime = 0;

    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        previousTime = getRuntime();

        while (opModeIsActive()) {

            double targetVelocity = 740;
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
