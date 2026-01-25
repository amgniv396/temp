package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.shootCommandGroup;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruOpMode;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Commands.intakeCommandGroup;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

@TeleOp
public class manualDrive extends JeruOpMode {

    public JeruRobot robotInstance;

    @Override
    public void initialize() {
        robotInstance = JeruRobot.getInstance();
        robotInstance.initJeruRobot()
                .angle(0)
                .allianceColor(AllianceColor.BLUE)
                .opModeType(OpModeType.EXPERIMENTING_NO_EXPANSION)
                .build(this);

        //Drive
        new Trigger(() -> JeruRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whileActiveContinuous(
                DriveTrain.getInstance().slowmodeFieldOrientedDriveCommand()
        );
        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                DriveTrain.getInstance().resetYawCommand()
        );

        //Intake
        new Trigger(() -> robotInstance.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whileActiveOnce(
                intakeCommandGroup.intakeCommand()
        ).whenInactive(
                intakeCommandGroup.disableIntakeSystems()
        );
        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                intakeCommandGroup.intakeCommand(),
                intakeCommandGroup.disableIntakeSystems()
        );

        //Shooter
        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                shootCommandGroup.shootAll()
        );
        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                shootCommandGroup.disableShooterSubsystems()
        );
    }
}
