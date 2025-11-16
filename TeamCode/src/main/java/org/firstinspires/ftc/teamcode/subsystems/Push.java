package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Push {
    private Servo push;
    public static final double pushPower = 0;
    public static final double setPosition = 0;
    public Push(HardwareMap hardwareMap) {
        push = hardwareMap.get(Servo.class, "Push");
    }

    class PushBallUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                push.setPosition(pushPower);
            }
            return false;
        }


    }
    class PushBallDown implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                push.setPosition(0);
            }
            return false;
        }

    }

    //hi
    public Action PushBallUp() {
        return new PushBallUp();
    }
    public Action PushBallDown(){
        return new PushBallDown();
    }
}
