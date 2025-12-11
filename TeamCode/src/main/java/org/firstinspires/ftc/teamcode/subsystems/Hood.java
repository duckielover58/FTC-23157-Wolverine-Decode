package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hood {
    private CRServo hood;
    public Hood (HardwareMap hardwareMap) {
        hood = hardwareMap.get(CRServo.class, "Hood");
    }

    public class HoodPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //hood.setPosition(0.5);
            return false;
        }
    }

    public class HoodUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPower(0.01);
            return false;
        }
    }

    public class HoodDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPower(-0.01);
            return false;
        }
    }

    public Action HoodPosition() {
        return new HoodPosition();
    }
    public Action hoodDown() {
        return new HoodDown();
    }
    public Action hoodUp() {
        return new HoodUp();
    }

    /*
    public Action hoodPos() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hood.setPosition(0.5);
                return false;
            }
        };
    }
    */

}
