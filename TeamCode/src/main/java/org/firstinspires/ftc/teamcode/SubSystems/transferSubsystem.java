package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

public class transferSubsystem extends SubsystemBase {
    private final CuttleMotor transferMotor;
    private static transferSubsystem instance;

    public static synchronized transferSubsystem getInstance() {
        if (instance == null) {
            instance = new transferSubsystem();
        }
        return instance;
    }

    private transferSubsystem() {
        transferMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
    }

    private void setPower(double power) {
        transferMotor.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }
}
