package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.R;

public class Claw extends SubsystemBase {
    private final Servo clawServo;
    public static double open = 0.5;
    public static double close = 0;

    public Claw() {
        clawServo = hardwareMap.get(Servo.class, "claw");
    }

    public Command grab() {
        return new InstantCommand(() -> clawServo.setPosition(close),this);
    }

    public Command release() {
        return new InstantCommand(() -> clawServo.setPosition(open),this);
    }

    public Command setPosition(double pos) {
        return new InstantCommand(() -> clawServo.setPosition(pos),this);
    }
}
