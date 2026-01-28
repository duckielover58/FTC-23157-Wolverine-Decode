package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.currentBallPos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Index {

    private Servo index;
    private double outtakeBall1 = 0.19; //between 0.15-0.2
    private double outtakeBall2 = 0.55; //0.40
    private double outtakeBall3 = 0.92; //between 0.85 - 0.9
    private double intakeBall1 = 0.00;
    private double intakeBall2 = 0.37;
    private double intakeBall3 = 0.74;

    public Index (HardwareMap hardwareMap) {
        index = hardwareMap.get(Servo.class, "Index");
    }

    public class intakeIndex1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(intakeBall1);
            currentBallPos = 1;
            return false;
        }
    }
    public class OuttakeIndex1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(outtakeBall1);
            currentBallPos = 1;
            return false;
        }
    }

public class intakeIndex2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(intakeBall2);
            currentBallPos = 2;
            return false;
        }
    }
    public class outtakeIndex2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(outtakeBall2);
            currentBallPos = 2;
            return false;
        }
    }

public class intakeIndex3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(intakeBall3);
            currentBallPos = 3;
            return false;
        }
    }

    public class outtakeIndex3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(outtakeBall3);
            currentBallPos = 3;
            return false;
        }
    }
    public class intakeIndexHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            index.setPosition(0);
            currentBallPos = 0;
            return false;
        }
    }

    public class intakeTurnRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == intakeBall1 || index.getPosition() == 0) {
                index.setPosition(intakeBall2);
                currentBallPos = 2;
            }
            if (index.getPosition() == intakeBall2) {
                index.setPosition(intakeBall3);
                currentBallPos = 3;
            }
            if (index.getPosition() == intakeBall3) {
                index.setPosition(intakeBall1);
                currentBallPos = 1;
            }
            return false;
        }
    }

    public class outtakeTurnRight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == outtakeBall1 || index.getPosition() == 0) {
                index.setPosition(outtakeBall2);
                currentBallPos = 2;
            }
            if (index.getPosition() == outtakeBall2) {
                index.setPosition(outtakeBall3);
                currentBallPos = 3;
            }
            if (index.getPosition() == outtakeBall3) {
                index.setPosition(outtakeBall1);
                currentBallPos = 1;
            }
            return false;
        }
    }

    public class intakeTurnLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == intakeBall3) {
                index.setPosition(intakeBall2);
                currentBallPos = 2;
            }
            if (index.getPosition() == intakeBall2) {
                index.setPosition(intakeBall1);
                currentBallPos = 1;
            }
            if (index.getPosition() == intakeBall1 || index.getPosition() == 0) {
                index.setPosition(intakeBall3);
                currentBallPos = 3;
            }
            return false;
        }
    }

    public class outtakeTurnLeft implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getPosition() == outtakeBall3) {
                index.setPosition(outtakeBall2);
                currentBallPos = 2;
            }
            if (index.getPosition() == outtakeBall2) {
                index.setPosition(outtakeBall1);
                currentBallPos = 1;
            }
            if (index.getPosition() == outtakeBall1 || index.getPosition() == 0) {
                index.setPosition(outtakeBall3);
                currentBallPos = 3;
            }
            return false;
        }
    }

    public class ReverseDirection implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (index.getDirection() == Servo.Direction.REVERSE) {
                index.setDirection(Servo.Direction.FORWARD);
            } else {
                index.setDirection(Servo.Direction.REVERSE);
            }

            return false;
        }
    }

    public Action intakeIndex1() { return new Index.intakeIndex1(); }
    public Action outtakeIndex1() { return new OuttakeIndex1(); }
    public Action intakeIndex2() { return new Index.intakeIndex2(); }
    public Action outtakeIndex2() { return new Index.outtakeIndex2(); }
    public Action intakeIndex3() { return new Index.intakeIndex3(); }
    public Action outtakeIndex3() { return new Index.outtakeIndex3(); }
    public Action indexHome() { return new Index.intakeIndexHome(); }
    public Action intakeTurnRight() { return new Index.intakeTurnRight(); }
    public Action reverseDirection() { return new Index.ReverseDirection(); }
    public Action outtakeTurnRight() { return new Index.outtakeTurnRight(); }
    public Action intakeTurnLeft() { return new Index.intakeTurnLeft(); }
    public Action outtakeTurnLeft() { return new Index.outtakeTurnLeft(); }

}
