package org.firstinspires.ftc.teamcode.subsystems;
import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.currentBallPos;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.seeyuhHSV;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.ball1Color;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.ball2Color;
import static org.firstinspires.ftc.teamcode.subsystems.GlobalVariable.ball3Color;

public class Seeyuh {

    private ColorSensor seeyuh;
    int Green = 0;
    int Purple = 1;

    public Seeyuh (HardwareMap hardwareMap) { hardwareMap.get(ColorSensor.class, "Seeyuh"); }

    public class Detect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Color.RGBToHSV(seeyuh.red(), seeyuh.green(), seeyuh.blue(), seeyuhHSV);

            if (seeyuhHSV[0] >= 100 && seeyuhHSV[0] <= 140) {
                if (currentBallPos == 1) {
                    ball1Color = Green;
                } else if (currentBallPos == 2) {
                    ball2Color = Green;
                } else {
                    ball3Color = Green;
                }
            } else if (seeyuhHSV[0] >= 270 && seeyuhHSV[0] <= 340) {
                if (currentBallPos == 1) {
                    ball1Color = Purple;
                } else if (currentBallPos == 2) {
                    ball2Color = Purple;
                } else {
                    ball3Color = Purple;
                }
            }
            return false;
        }
    }

    public Action detect () { return new Detect(); }

}
