package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Index {

    private Servo index;
    private double ball1 = 0.02; //0.05
    private double ball2 = 0.39; //0.40
    private double ball3 = 0.75; //0.75

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
    public class IndexHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(0);
            return false;
        }
    }

    public class TurnRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == ball1) {
                index.setPosition(ball2);
            }
            if (index.getPosition() == ball2) {
                index.setPosition(ball3);
            }
            if (index.getPosition() == ball3) {
                index.setPosition(ball1);
            }
            return false;
        }
    }

    public class TurnLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == ball3) {
                index.setPosition(ball2);
            }
            if (index.getPosition() == ball2) {
                index.setPosition(ball1);
            }
            if (index.getPosition() == ball1) {
                index.setPosition(ball3);
            }
            return false;
        }
    }

    public Action index1() { return new Index.Index1(); }
    public Action index2() { return new Index.Index2(); }
    public Action index3() { return new Index.Index3(); }
    public Action indexHome() { return new Index.IndexHome(); }
    public Action turnRight() { return new Index.TurnRight(); }
    public Action turnLeft() { return new Index.TurnLeft(); }

}
