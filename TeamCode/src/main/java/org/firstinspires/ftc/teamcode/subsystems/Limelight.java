package org.firstinspires.ftc.teamcode.subsystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    private Limelight3A limelight;
    public static double ServoPower = 1;
    public boolean servoLocked = false;

    // tuning constants
    private final double bearingThreshold = 3;
    private static double servoSpeed = 0.1;
    private CRServo swivel;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        swivel = hardwareMap.get(CRServo.class, "Swivel");
    }

    public class LimelightRed implements Action {

        private boolean servoLocked = false;
        private boolean pipelineSet = false; // ensure pipeline switches only once

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            // Set the pipeline once at the start
            if (!pipelineSet) {
                limelight.pipelineSwitch(0);
                pipelineSet = true;
            }

            if (servoLocked) {
                swivel.setPower(0);
                packet.put("locked", true);
                return false;
            }

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                swivel.setPower(0);
                packet.put("targetValid", false);
                return true; //keep trying until target valid
            }

            double bearing = result.getTx();

            if (bearing > bearingThreshold) {
                swivel.setPower(-servoSpeed);
            } else if (bearing < -bearingThreshold) {
                swivel.setPower(servoSpeed);
            } else {
                swivel.setPower(0);
                servoLocked = true; // lock complete
            }
            packet.put("bearing", bearing);
            packet.put("servoPower", swivel.getPower());
            packet.put("locked", servoLocked);
            packet.put("targetValid", result.isValid());

            return !servoLocked; // keep running until locked
        }
    }
    public class LimelightBlue implements Action {

        private boolean servoLocked = false;
        private boolean pipelineSet = false; // ensure pipeline switches only once

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            // Set the pipeline once at the start
            if (!pipelineSet) {
                limelight.pipelineSwitch(2);
                pipelineSet = true;
            }

            if (servoLocked) {
                swivel.setPower(0);
                packet.put("locked", true);
                return false;
            }

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                swivel.setPower(0);
                packet.put("targetValid", false);
                return true; //keep trying until target valid
            }

            double bearing = result.getTx();

            if (bearing > bearingThreshold) {
                swivel.setPower(-servoSpeed);
            } else if (bearing < -bearingThreshold) {
                swivel.setPower(servoSpeed);
            } else {
                swivel.setPower(0);
                servoLocked = true; // lock complete
            }
            packet.put("bearing", bearing);
            packet.put("servoPower", swivel.getPower());
            packet.put("locked", servoLocked);
            packet.put("targetValid", result.isValid());

            return !servoLocked; // keep running until locked
        }
    }
    public Action limelightRed() { return new Limelight.LimelightRed(); }
    public Action limelightBlue(){ return new Limelight.LimelightBlue(); }

}

