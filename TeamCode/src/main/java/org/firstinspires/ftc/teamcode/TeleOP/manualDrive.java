package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Commands.intakeCommandGroup;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

@TeleOp
public class manualDrive extends CommandOpMode {

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
        JeruRobot.getInstance().expansionHub.pullBulkData();
    }

    @Override
    public void end() {
        JeruRobot.getInstance().resetRobot();
    }
}
