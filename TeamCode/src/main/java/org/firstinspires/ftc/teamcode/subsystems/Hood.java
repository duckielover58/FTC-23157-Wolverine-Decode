package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.testingFiles.MainTeleOp;

public class Hood {
    private Servo hood;
    public double hoodPos;
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

    public class HoodUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hoodPos = hood.getPosition();
            hoodPos += 0.1;
            hood.setPosition(hoodPos);
            return false;
        }
    }

    public class HoodDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hoodPos = hood.getPosition();
            hoodPos -= 0.1;
            hood.setPosition(hoodPos);
            return false;
        }
    }

    public class HoodPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            MainTeleOp.hoodPoss = hood.getPosition();
            return false;
        }
    }

    public Action hoodPosition() {
        return new HoodPosition();
    }
    public Action hoodDown() {
        return new HoodDown();
    }
    public Action hoodUp() {
        return new HoodUp();
    }

    public Action hoodPos() {
        return new HoodPos();
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
