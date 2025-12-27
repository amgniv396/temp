package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.ClawSubSystem;

public class intakeCommandGroup {
    public static Command prepareIntake() {
        return new ParallelCommandGroup(
                ArmSubSystem.getInstance().setPositionCommand(ArmSubSystem.middle),
                ClawSubSystem.getInstance().release()
        );
    }

    public static Command Intake() {
        return new SequentialCommandGroup(
                ArmSubSystem.getInstance().setPositionCommand(ArmSubSystem.down),
                new WaitCommand(200),
                ClawSubSystem.getInstance().grab(),
                new WaitCommand(200),
                ArmSubSystem.getInstance().setPositionCommand(ArmSubSystem.up)
        );
    }
}
