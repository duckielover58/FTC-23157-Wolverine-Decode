package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FlywheelController {

    private final DcMotor motor;

    // tuning variables
    private final double kP = 0.0008;  // proportional gain
    private final double basePower = -0.75;  // minimum power to keep flywheel spinning
    private final double ticksPerMs = -0.9288;   // 928.8 ticks/sec = 0.9288 ticks/ms

    // internal state
    private double targetTicks = 0;
    private long startTime = 0;
    public boolean atSpeed = false;

    public FlywheelController(DcMotor m) {
        this.motor = m;
    }

    public void start() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startTime = System.currentTimeMillis();
        targetTicks = 0;
        atSpeed = false;
    }

    public void stop() {
        motor.setPower(0);
        atSpeed = false;
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - startTime;

        targetTicks = ticksPerMs * elapsed;
        double currentTicks = motor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        // Proportional correction
        double power = basePower + kP * error;

        // Clamp for safety
        power = Math.max(-1, Math.min(1, power));

        motor.setPower(power);

        atSpeed = Math.abs(error) < 150;
    }
}
