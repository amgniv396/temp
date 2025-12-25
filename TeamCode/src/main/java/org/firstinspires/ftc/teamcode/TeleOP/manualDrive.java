package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.onbotjava.handlers.file.TemplateFile;
import org.firstinspires.ftc.teamcode.JeruRobot;
import org.firstinspires.ftc.teamcode.Commands.intakeCommandGroup;

@TeleOp
public class manualDrive extends CommandOpMode {

    public JeruRobot robotInstance;

    @Override
    public void initialize() {
        robotInstance = JeruRobot.getInstance();

        robotInstance.initBarnRobot(this);

        robotInstance.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                intakeCommandGroup.prepareIntake()
        );

        new Trigger(() -> robotInstance.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whileActiveOnce(
                intakeCommandGroup.Intake()
        );
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void end() {
        JeruRobot.getInstance().resetRobot();
    }
}
