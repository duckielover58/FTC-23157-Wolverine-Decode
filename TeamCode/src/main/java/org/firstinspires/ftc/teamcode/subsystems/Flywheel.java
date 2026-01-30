package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.close;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.far;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.kF;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.postIntake100;
import static org.firstinspires.ftc.teamcode.testingFiles.Flywheelgm0PIDtest.kP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainTeleOp;
public class Flywheel {
    private DcMotorEx flywheel;
    double shootPower = close - 300;

    public Flywheel (HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel2");
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
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

    public class GetV implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            MainTeleOp.flywheelV = flywheel.getVelocity();
            return false;
        }
    }

    public class PIDPinit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelPID(shootPower);
            if (postIntake100) {
                return true;
            } else {
                flywheel.setPower(0);
                return false;
            }
        }
    }

    public class PIDP1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelPID(shootPower);
            if (postIntake100) {
                return true;
            } else {
                flywheel.setPower(0);
                return false;
            }
        }
    }

    public class PIDP2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelPID(far);
            if (postIntake100) {
                return true;
            } else {
                flywheel.setPower(0);
                return false;
            }
        }
    }

    public class FlywheelInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheel.setPower(0.1);
            new SleepAction(0.5);
            flywheel.setPower(0);
            return false;
        }
    }
    public Action shoot() { return new Shoot(); }
    public Action shootStop() { return new ShootStop(); }
    public Action getV() { return new GetV(); }
    public Action PIDp1() { return new PIDP1(); }
    public Action PIDp2() { return new PIDP2(); }
    public Action flywheelInit() { return new FlywheelInit(); }
    void flywheelPID (double target) {

        double currentVelocity = flywheel.getVelocity();
        double error = target - currentVelocity;

        double ff = kF * target;

        double output = ff + (kP * error);

        output = Math.max(0.0, Math.min(1.0, output));

        flywheel.setPower(output);
    }

}
