package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Flywheel {
    private DcMotor flywheel;

    public Flywheel (HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    public class Shoot implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flywheel.setPower(1);
            }
            return false;
        }
    }

    public class ShootStop implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flywheel.setPower(0);
            }
            return false;
        }
    }
    public Action shoot() { return new Shoot(); }
    public Action shootStop() { return new ShootStop(); }


}
