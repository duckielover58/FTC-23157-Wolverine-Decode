package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearing;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.bearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.blueTag;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.lockSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.maxBearingErr;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.redTag;

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
    private CRServo swivel = hardwareMap.get(CRServo.class, "Swivel");
    private boolean ServoLocked = false;

    public LimelightCam (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public class LodkRed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limelight.start();
            ServoLocked = false;
            if (!ServoLocked) {
                lodk1(redTag);
                return true;
            }
            return false;
        }
    }

    public class LodkBlue implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limelight.start();
            ServoLocked = false;
            if (!ServoLocked) {
                lodk1(blueTag);
                return true;
            }
            return false;
        }
    }

    public Action lodkRed() { return new LodkRed(); }
    public Action lodkBlue() { return new LodkBlue(); }
    void lodk1 (int tag) {
        telemetry.addLine("Started");
        limelight.pipelineSwitch(tag);
        LLResult result = limelight.getLatestResult();
        telemetry.addLine("Result?");
        if (result != null) {
            if (result.isValid()) {
                telemetry.addLine("In Loop");
                bearing = result.getTx();
                bearingErr = bearing - maxBearingErr;
                lockSpeed = 0.1 * bearingErr;
                lockSpeed = Math.max(-0.3, Math.min((lockSpeed), 0.3));
                if (bearing > maxBearingErr - 0.5) {
                    swivel.setDirection(CRServo.Direction.FORWARD);
                    swivel.setPower(-lockSpeed);
                } else if (bearing < -maxBearingErr + 0.5) {
                    swivel.setDirection(CRServo.Direction.REVERSE);
                    swivel.setPower(lockSpeed);
                } else {
                    swivel.setPower(0);
                    ServoLocked = true;
                }
            } else swivel.setPower(0);
        } else swivel.setPower(0);
    }

}
