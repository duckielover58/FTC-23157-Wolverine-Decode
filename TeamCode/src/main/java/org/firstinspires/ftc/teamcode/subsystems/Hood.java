package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hood {
    private Servo hood;
    public Hood (HardwareMap hardwareMap) {
        hood = hardwareMap.get(Servo.class, "Hood");
    }

    public class HoodPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.5);
            return false;
        }
    }

    public Action HoodPosition() {
        return new HoodPosition();
    }

}
