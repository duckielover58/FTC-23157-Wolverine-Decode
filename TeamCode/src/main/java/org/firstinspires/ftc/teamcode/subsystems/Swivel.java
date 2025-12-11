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

    public Action aim () {
        return new Swivel.Aim();
    }
    public Action targetBlue() { return new TargetBlue(); }
    public Action targetRed() { return new TargetRed(); }
}
