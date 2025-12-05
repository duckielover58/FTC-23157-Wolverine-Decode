package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    public double intakePower = 1;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "Intake");
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

    class OuttakeBall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(-intakePower);
            }
            return false;
        }
    }

    class IntakeBallStop implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(0);
            }
            return false;
        }


    }
    public Action IntakeBall() { return new IntakeBall(); }
    public Action OuttakeBall() { return new OuttakeBall(); }
    public Action IntakeBallStop (){
        return new IntakeBallStop();
    }

}

//hello
