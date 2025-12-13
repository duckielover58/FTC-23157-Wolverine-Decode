package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Swivel {

    private double blueTarget;
    private double redTarget;
    private double servoPower;

    private CRServo swivel;
    //private Servo swivel;

    public Swivel (HardwareMap hardwareMap) {
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        //swivel = hardwareMap.get(Servo.class, "Swivel");
    }

    public void setPower(double servoPower) {
        this.servoPower = servoPower;
    }


    public class Aim implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(servoPower);
            return false;
        }
    }

    public class TargetBlue implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //swivel.setPosition(blueTarget);
            return false;
        }
    }

    public class TargetRed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //swivel.setPosition(redTarget);
            return false;
        }
    }

    public class Turn90right implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(-1);
            return false;
        }
    }

    public class Turn90left implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(1);
            return false;
        }
    }

    public class StopSwivel implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(0);
            return false;
        }
    }

    public Action aim () {
        return new Aim();
    }
    public Action turn90right() { return new Turn90right(); }
    public Action turn90left() { return new Turn90left(); }
    public Action targetBlue() { return new TargetBlue(); }
    public Action targetRed() { return new TargetRed(); }
}
