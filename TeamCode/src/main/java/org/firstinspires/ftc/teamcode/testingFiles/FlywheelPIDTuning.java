package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutionException;

@TeleOp(name = "FlywheelPIDTuning")
public class FlywheelPIDTuning extends LinearOpMode {

    private DcMotor flywheel;
    public double powerMultiplier = 0.001;
    private double flywheelPower = -0.75;
    private double flywheelTicksPerMilliSec = -928.8 / 1000;
    private double flywheelTargetTicks;
    private double flywheelTicks;
    private double flywheelError;
    public double flywheelAllowedError = 100;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double pushPos = 0;
        double step = 0.1;
        long slep = 20;
        long rantime = 0;


        waitForStart();

        while (opModeIsActive()) {
            flywheel.setPower(flywheelPower);
            flywheelTargetTicks = flywheelTicksPerMilliSec * rantime;
            flywheelTicks = flywheel.getCurrentPosition();
            flywheelError = flywheelTargetTicks - flywheelTicks;
            telemetry.addData("Flywheel Power: ", flywheelPower);
            telemetry.addData("Flywheel Ticks: ", flywheelTicks);
            telemetry.addData("Flywheel Target Ticks: ", flywheelTargetTicks);
            telemetry.addData("Flywheel Tick Difference: ", flywheelTargetTicks - flywheelTicks);

            if (flywheelTargetTicks - flywheelAllowedError <= flywheelTicks && flywheelTicks <= flywheelTargetTicks + flywheelAllowedError) {
                telemetry.addLine("Flywheel Chilling");
            } else {
                flywheelPower += (powerMultiplier * flywheelError);
                flywheelPower = Math.max(-1, 1);
                telemetry.addData("Flywheel adjusted by: ", powerMultiplier * flywheelError);
            }
            telemetry.addData("Flywheel Ticks", flywheel.getCurrentPosition());
            telemetry.update();
            sleep(slep);
            rantime += slep;
        }
    }
}

