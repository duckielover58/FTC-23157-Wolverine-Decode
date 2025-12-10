package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@TeleOp
public class LimelightShooterLock extends LinearOpMode {

    private Limelight3A limelight;
    public static double ServoPower = 1;
    public boolean servoLocked = false;

    // tuning constants
    private final double bearingThreshold = 1; // same hting as bearing error
    private final double servoSpeed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // adjust index for our specific AprilTag


        Swivel swivel = new Swivel(hardwareMap);

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose(); // pose from Limelight
                double bearing = result.getTx(); // x offset in degrees from target (target x and bearing are the same thing i think)

                // servo aiming/locking
                if (!servoLocked) {
                    if (bearing > bearingThreshold) {
                        ServoPower = -servoSpeed;
                        Actions.runBlocking(swivel.aim());
                    } else if (bearing < -bearingThreshold) {
                        ServoPower = servoSpeed;
                        Actions.runBlocking(swivel.aim());
                    } else {
                        ServoPower = 0;
                        Actions.runBlocking(swivel.aim());
                        servoLocked = true;
                    }
                }

                // telemetry
                telemetry.addData("Bearing", bearing);
                telemetry.addData("Servo Power", ServoPower);
                telemetry.addData("Servo Locked", servoLocked);
                telemetry.addData("Target Valid", result.isValid());
                telemetry.update();
            }

            sleep(20);
        }
    }
}
