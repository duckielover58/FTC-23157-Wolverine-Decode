package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo intake;
    public double intakePower = 1;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "Intake");
    }

    class IntakeBall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(intakePower);
            }
            return false;
        }
    }
    public Action IntakeBall() {
        return new IntakeBall();
    }

}
