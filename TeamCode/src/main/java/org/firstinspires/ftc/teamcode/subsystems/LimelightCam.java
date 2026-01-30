package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LimelightCam {
    private Limelight3A limelight;
    boolean star = false;
//    private CRServo swivel = hardwareMap.get(CRServo.class, "Swivel");


    public LimelightCam (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public class LodkRed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!star) {
                limelight.start();
                star = true;
                ServoLocked = false;
            }
            if (!ServoLocked) {
                lodk1(redTag);
                return true;
            }
            limelight.stop();
            star = false;
            return false;
        }
    }

    public class LodkBlue implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!star) {
                limelight.start();
                star = true;
                ServoLocked = false;
            }
            if (!ServoLocked) {
                lodk1(blueTag);
                return true;
            }
            limelight.stop();
            star = false;
            return false;
        }
    }

    public class MotifCheck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            MotChk();
            limelight.stop();
            return false;
        }
    }

    public Action lodkRed() { return new LodkRed(); }
    public Action lodkBlue() { return new LodkBlue(); }
    public Action motifCheck() { return new MotifCheck(); }

    void MotChk () {
        LLResult result = limelight.getLatestResult();
        for (int i = 4; i < 7; i ++) {
            limelight.start();
            limelight.pipelineSwitch(i);
            if (result.isValid()) {
                motif = 17+i;
                break;
            }
        }
    }
    void lodk1 (int tag) {
        limelight.pipelineSwitch(tag);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                bearing = result.getTx();
                bearingErr = bearing - maxBearingErr;
                lockSpeed = 0.1 * bearingErr;
                lockSpeed = Math.max(-0.3, Math.min((lockSpeed), 0.3));
                if (bearing > maxBearingErr - 0.5) {
                    overUnder = 100;
//                    swivel.setDirection(CRServo.Direction.FORWARD);
//                    swivel.setPower(-lockSpeed);
                } else if (bearing < -maxBearingErr - 0.5) {
                    overUnder = -100;
//                    swivel.setDirection(CRServo.Direction.REVERSE);
//                    swivel.setPower(lockSpeed);
                } else {
                    overUnder = 0;
//                    swivel.setPower(0);
                    ServoLocked = true;
                }
            }
        }
    }
}
