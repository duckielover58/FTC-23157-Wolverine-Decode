package org.firstinspires.ftc.teamcode.testingFiles;

// RR-specific imports
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Push;
import org.firstinspires.ftc.teamcode.subsystems.Swivel;

@TeleOp
public class TestingActions extends LinearOpMode {
    Intake intake;
    Push push;
    Flywheel flywheel;
    Swivel swivel;
    Index index;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        push = new Push(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        swivel = new Swivel(hardwareMap);
        index = new Index(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Actions.runBlocking(intake.IntakeBall());
            }
            if (gamepad1.left_bumper) {
                Actions.runBlocking(intake.IntakeBallStop());
            }
            if (gamepad1.b) {
                Actions.runBlocking(push.PushBallUp());
            }
            if (gamepad1.dpad_down) {
                Actions.runBlocking(push.PushBallDown());
            }
            if (gamepad1.y) {
                Actions.runBlocking(flywheel.shoot());
            }
            if (gamepad1.x) {
                Actions.runBlocking(index.index1());
            }
            if (gamepad1.dpad_up) {
                Actions.runBlocking(index.index2());
            }
            if (gamepad1.dpad_left) {
                Actions.runBlocking(index.index3());
            }
            if (gamepad1.dpad_right) {
                Actions.runBlocking(swivel.aim());
            }
        }
    }
}

