package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.testingFiles.MainTeleOp;
public class Flywheel {
    private DcMotorEx flywheel;

    public Flywheel (HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public class Shoot implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flywheel.setPower(0.7);
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

    public class GetV implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            MainTeleOp.flywheelV = flywheel.getVelocity();
            return false;
        }
    }
    public Action shoot() { return new Shoot(); }
    public Action shootStop() { return new ShootStop(); }
    public Action getV() { return new GetV(); }

}
