package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.testingFiles.ShooterLockTest;

public class Swivel {

    private CRServo swivel;

    public Swivel (HardwareMap hardwareMap) {
        swivel = hardwareMap.get(CRServo.class, "Swivel");
    }

    public class Aim implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            swivel.setPower(ShooterLockTest.ServoPower);
            return false;
        }
    }

    public Action aim () {
        return new Swivel.Aim();
    }
}
