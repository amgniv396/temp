package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
@Config
public class ClawSubSystem extends SubsystemBase {
    private final Servo clawServo;
    public static double open = 0.5;
    public static double close = 0;
    private static ClawSubSystem instance;
    public static synchronized ClawSubSystem getInstance() {
        if (instance == null) {
            instance = new ClawSubSystem();
        }
        return instance;
    }

    private ClawSubSystem() {
        clawServo = JeruRobot.getInstance().hardwareMap.get(Servo.class, "claw");
    }

    private void setPosition(double pos) {
        clawServo.setPosition(pos);
    }

    public Command grab() {
        return new InstantCommand(() -> setPosition(close),this);
    }

    public Command release() {
        return new InstantCommand(() -> setPosition(open),this);
    }
}
