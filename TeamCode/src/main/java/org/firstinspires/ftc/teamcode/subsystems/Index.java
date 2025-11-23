package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Index {

    private Servo index;
    private double ball1 = 0.25;
    private double ball2 = 0.5;
    private double ball3 = 0.75;

    public Index (HardwareMap hardwareMap) {
        index = hardwareMap.get(Servo.class, "Index");
    }

    public class Index1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(ball1);
            return false;
        }
    }

public class Index2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(ball2);
            return false;
        }
    }

public class Index3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(ball3);
            return false;
        }
    }

    public Action index1() { return new Index.Index1(); }
    public Action index2() { return new Index.Index2(); }
    public Action index3() { return new Index.Index3(); }

}
