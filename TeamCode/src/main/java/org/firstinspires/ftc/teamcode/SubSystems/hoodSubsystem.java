package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

public class hoodSubsystem extends SubsystemBase {
    private final CuttleServo hoodServo;
    private static hoodSubsystem instance;

    public static synchronized hoodSubsystem getInstance() {
        if (instance == null) {
            instance = new hoodSubsystem();
        }
        return instance;
    }

    private hoodSubsystem() {
        hoodServo = new CuttleServo(JeruRobot.getInstance().controlHub, 3);
    }

    private void setPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public Command setPositionCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos),this);
    }
    public Command disableSystem() {
        return new InstantCommand(()->{},this);
    }
}
