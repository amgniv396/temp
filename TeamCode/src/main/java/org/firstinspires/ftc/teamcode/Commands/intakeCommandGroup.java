package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.ArmsSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.ArmsSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSubSystem;

public class intakeCommandGroup {
    public static Command intakeCommand() {
        return new ParallelCommandGroup(
                IntakeSubSystem.getInstance().setPowerCommand(1),
                ArmsSubSystem.getInstance().setPositionCommand(ArmsSubSystem.closed)
        );
    }
}
