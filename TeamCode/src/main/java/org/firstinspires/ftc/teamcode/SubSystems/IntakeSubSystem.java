package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

public class IntakeSubSystem extends SubsystemBase {
    private final CuttleMotor intakeMotor;
    private static IntakeSubSystem instance;

    public static synchronized IntakeSubSystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubSystem();
        }
        return instance;
    }

    private IntakeSubSystem() {
        intakeMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
    }

    private void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }
}
