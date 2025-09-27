package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.BarnRobot;
import org.firstinspires.ftc.teamcode.Commands.intakeCommandGroup;

@TeleOp
public class manualDrive extends CommandOpMode {

    public BarnRobot robotInstance;

    @Override
    public void initialize() {
        robotInstance = BarnRobot.getInstance();

        robotInstance.initBarnRobotSystems();

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
}
