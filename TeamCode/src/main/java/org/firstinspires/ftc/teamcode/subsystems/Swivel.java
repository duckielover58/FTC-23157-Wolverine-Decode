package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.testingFiles.ShooterLockTest;

public class Swivel {

    private double blueTarget;
    private double redTarget;

    private CRServo swivel;
    //private Servo swivel;

    public Swivel (HardwareMap hardwareMap) {
        swivel = hardwareMap.get(CRServo.class, "Swivel");
        //swivel = hardwareMap.get(Servo.class, "Swivel");
    }

    public class Aim implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(ShooterLockTest.ServoPower);
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
