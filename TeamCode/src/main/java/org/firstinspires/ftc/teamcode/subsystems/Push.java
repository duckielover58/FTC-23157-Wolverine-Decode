package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Push {
    private Servo push;
    public static final double upPos = 0.4;
    public static final double downPos = 0;

    //push max 0.95
    public Push(HardwareMap hardwareMap) {
        push = hardwareMap.get(Servo.class, "Push");
    }

    class PushBallUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                push.setPosition(upPos);
            }
            return false;
        }
    }

    class PushBallDown implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                push.setPosition(downPos);
            }
            return false;
        }
    }

    public Action PushBallUp() {
        return new PushBallUp();
    }
    public Action PushBallDown(){
        return new PushBallDown();
    }
}
