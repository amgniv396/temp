package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.BarnRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class intakeCommandGroup {
    public static Command prepareIntake() {
        return new ParallelCommandGroup(
                BarnRobot.getInstance().arm.setPosition(Arm.middle),
                BarnRobot.getInstance().claw.release()
        );
    }

    public static Command Intake() {
        return new SequentialCommandGroup(
                BarnRobot.getInstance().arm.setPosition(Arm.down),
                new WaitCommand(200),
                BarnRobot.getInstance().claw.grab(),
                new WaitCommand(200),
                BarnRobot.getInstance().arm.setPosition(Arm.up)
        );
    }
}
