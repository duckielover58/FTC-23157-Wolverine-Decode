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
    private IMU imu;
    public static double ServoPower = 0;
    public boolean servoLocked = false;

    // tuning constants
    private final double bearingThreshold = 3; // same hting as bearing error
    private final double servoSpeed = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        limelight.pipelineSwitch(0); // adjust index for our specific AprilTag

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientation));

        Swivel swivel = new Swivel(hardwareMap);

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // Updates Limelight orientation
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2(); // pose from Limelight
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
