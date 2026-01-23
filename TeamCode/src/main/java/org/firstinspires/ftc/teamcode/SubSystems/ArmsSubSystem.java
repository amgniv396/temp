package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

@Config
public class ArmsSubSystem extends SubsystemBase {
    private final CuttleServo rightServo;
    private final CuttleServo leftServo;
    public static double open = 0.5;
    public static double closed = 0.25;

    private static ArmsSubSystem instance;

    public static synchronized ArmsSubSystem getInstance() {
        if (instance == null) {
            instance = new ArmsSubSystem();
        }
        return instance;
    }

    private ArmsSubSystem() {
        rightServo = new CuttleServo(JeruRobot.getInstance().controlHub, 3);
        leftServo = new CuttleServo(JeruRobot.getInstance().controlHub, 4);
        rightServo.setDirection(Direction.REVERSE);

        setPosition(closed);
    }

    private void setPosition(double pos) {
        rightServo.setPosition(pos);
        leftServo.setPosition(pos);
    }

    public Command setPositionCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos),this);
    }
}
