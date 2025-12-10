package org.firstinspires.ftc.teamcode.testingFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public boolean goalReached = false;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            telemetry.addData("Goal Reached? ", goalReached);

            if (flywheelTargetTicks <= flywheelTicks + flywheelAllowedError && flywheelTicks - flywheelAllowedError <= flywheelTargetTicks) {
                flywheelPower += (powerMultiplier * flywheelError);
                flywheelPower = Math.max(-1.0, Math.min(1.0, flywheelPower));
                telemetry.update();
                goalReached = false;
            } else {
                goalReached = true;
            }
            telemetry.addData("Flywheel Ticks", flywheel.getCurrentPosition());
            telemetry.update();
            sleep(slep);
            rantime += slep;
        }
    }
}

