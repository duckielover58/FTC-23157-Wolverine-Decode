package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.shortHoodPos;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.velHoodPos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainTeleOp;

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

    public class SetHoodPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (velHoodPos != -100) {
                hood.setPosition(velHoodPos);
            }
            return false;
        }
    }

    public class HoodPosShortShoot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(shortHoodPos);
            return false;
        }
    }

    public class one implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.1);
            return false;
        }
    }

    public class two implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.2);
            return false;
        }
    }

    public class three implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.3);
            return false;
        }
    }

    public class four implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.4);
            return false;
        }
    }

    public class five implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.5);
            return false;
        }
    }

    public class six implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.6);
            return false;
        }
    }

    public class seven implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.7);
            return false;
        }
    }

    public class eight implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.8);
            return false;
        }
    }

    public class nine implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(0.9);
            return false;
        }
    }

    public class ten implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hood.setPosition(1);
            return false;
        }
    }


    public Action hoodPositionInit() { return new HoodPosition(); }
    public Action hoodDown() { return new HoodDown(); }
    public Action hoodUp() { return new HoodUp(); }
    public Action setHoodPosShoot() { return new SetHoodPos(); }
    public Action hoodPosTelemetry() { return new HoodPos(); }
    public Action shortHoodPos() { return new HoodPosShortShoot(); }

    /*
    public Action hoodPosTelemetry() {
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
