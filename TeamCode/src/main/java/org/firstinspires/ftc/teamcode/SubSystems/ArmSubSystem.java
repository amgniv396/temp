package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

@Config
public class ArmSubSystem extends SubsystemBase {
    private final CuttleServo rightServo;
    private final CuttleServo leftServo;
    public static double up = 0.5;
    public static double middle = 0.25;
    public static double down = 0;

    private static ArmSubSystem instance;

    public static synchronized ArmSubSystem getInstance() {
        if (instance == null) {
            instance = new ArmSubSystem();
        }
        return instance;
    }

    private ArmSubSystem() {
        rightServo = new CuttleServo(JeruRobot.getInstance().controlHub, 3);
        leftServo = new CuttleServo(JeruRobot.getInstance().controlHub, 4);
        rightServo.setDirection(Direction.REVERSE);


        setPosition(down);
    }

    private void setPosition(double pos) {
        rightServo.setPosition(pos);
        leftServo.setPosition(pos);
    }

    public Command setPositionCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos),this);
    }
}
