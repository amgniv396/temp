package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private final Servo rightServo;
    private final Servo leftServo;
    public static double up = 0.5;
    public static double middle = 0.25;
    public static double down = 0;

    public Arm() {
        rightServo = hardwareMap.get(Servo.class, "armRightServo");
        leftServo = hardwareMap.get(Servo.class, "armLeftServo");
    }

    public Command setPosition(double pos) {
        return new InstantCommand(() -> {
            rightServo.setPosition(pos);
            leftServo.setPosition(1-pos);
        },this);
    }
}
