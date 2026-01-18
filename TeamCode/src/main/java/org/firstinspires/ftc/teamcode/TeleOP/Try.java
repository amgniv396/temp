package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruOpMode;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
@TeleOp
public class Try extends JeruOpMode {
    public JeruRobot robotInstance;
    @Override
    public void initialize() {
        robotInstance = JeruRobot.getInstance();
        robotInstance.initJeruRobot()
                .angle(0)
                .allianceColor(AllianceColor.BLUE)
                .opModeType(OpModeType.EXPERIMENTING_NO_EXPANSION)
                .build(this);

        new Trigger(() -> JeruRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whileActiveContinuous(
                DriveTrain.getInstance().slowmodeFieldOrientedDriveCommand()
        );
        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                DriveTrain.getInstance().resetYawCommand()
        );

    }
}
