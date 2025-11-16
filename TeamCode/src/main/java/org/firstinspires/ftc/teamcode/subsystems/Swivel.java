package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Swivel {

    public double aimPos; //move this into another global file

    private Servo swivel;

    public Swivel (HardwareMap hardwareMap) {
        swivel = hardwareMap.get(Servo.class, "swivel");
    }

    public class Aim implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPosition(aimPos);
            return false;
        }
    }

    public Action aim () {
        return new Swivel.Aim();
    }
}
