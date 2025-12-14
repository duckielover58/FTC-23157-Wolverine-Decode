package org.firstinspires.ftc.teamcode.testingFiles;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@Disabled
@TeleOp
public class LimelightShooterLockBlue extends LinearOpMode {

    private Limelight3A limelight;
    public static double ServoPower = 1;
    public boolean servoLocked = false;

    // tuning constants
    private final double bearingThreshold = 3; // same hting as bearing error
    private static double servoSpeed = 0.1;
    boolean star = false;

    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();
        limelight.pipelineSwitch(2); // adjust index for our specific AprilTag
        CRServo swivel = hardwareMap.get(CRServo.class, "Swivel");
        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                servoSpeed -= 0.1;
                telemetry.addData("Speed ", servoSpeed);
                telemetry.update();
                sleep(150);
            }
            if (gamepad1.dpad_up) {
                servoSpeed += 0.1;
                telemetry.addData("Speed ", servoSpeed);
                telemetry.update();
                sleep(150);
            }
            if (gamepad1.a) {
                star = true;
            }



            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && !servoLocked && star) {
                Pose3D botpose = result.getBotpose(); // pose from Limelight
                double bearing = result.getTx(); // x offset in degrees from target (target x and bearing are the same thing i think)

                // servo aiming/locking
                if (bearing > bearingThreshold) {
                    ServoPower = -servoSpeed;
                } else if (bearing < -bearingThreshold) {
                    ServoPower = servoSpeed;
                } else {
                    ServoPower = 0;
                    servoLocked = true;
                    star = true;
                }

                swivel.setPower(ServoPower);

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