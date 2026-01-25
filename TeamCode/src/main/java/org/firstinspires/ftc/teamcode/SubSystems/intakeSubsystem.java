package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

public class intakeSubsystem extends SubsystemBase {
    private final CuttleMotor intakeMotor;
    private static intakeSubsystem instance;

    public static synchronized intakeSubsystem getInstance() {
        if (instance == null) {
            instance = new intakeSubsystem();
        }
        return instance;
    }

    private intakeSubsystem() {
        intakeMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
    }

    private void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }
}
