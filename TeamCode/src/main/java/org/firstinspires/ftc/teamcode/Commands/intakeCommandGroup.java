package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.JeruRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ArmSubSystem;

public class intakeCommandGroup {
    public static Command prepareIntake() {
        return new ParallelCommandGroup(
                JeruRobot.getInstance().arm.setPositionCommand(ArmSubSystem.middle),
                JeruRobot.getInstance().claw.release()
        );
    }

    public static Command Intake() {
        return new SequentialCommandGroup(
                JeruRobot.getInstance().arm.setPositionCommand(ArmSubSystem.down),
                new WaitCommand(200),
                JeruRobot.getInstance().claw.grab(),
                new WaitCommand(200),
                JeruRobot.getInstance().arm.setPositionCommand(ArmSubSystem.up)
        );
    }
}
