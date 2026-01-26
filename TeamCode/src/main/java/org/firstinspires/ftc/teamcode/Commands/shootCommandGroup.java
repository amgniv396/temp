package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.SubSystems.armsSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.transferSubsystem;

public class shootCommandGroup {
    public static Command shootAll() {
        return new ParallelRaceGroup(
                prepareToShoot(),
                shootWithoutPrepare().withTimeout(1000)
        ).andThen(
                disableShooterSubsystems()
        );
    }

    public static Command prepareToShoot() {
        return new ParallelRaceGroup(
                shooterSubsystem.getInstance().setDistanceBasedRPM(),
                turretSubsystem.getInstance().targetAtGoalWhileDriving(),
                hoodSubsystem.getInstance().setPositionBasedDisAndVal()
        );
    }
    public static Command shootWithoutPrepare() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooterSubsystem.getInstance().atSetPoint()),
                new ParallelCommandGroup(
                        armsSubsystem.getInstance().setPositionCommand(armsSubsystem.open),
                        intakeSubsystem.getInstance().setPowerCommand(1),
                        transferSubsystem.getInstance().setPowerCommand(1)
                )
        );
    }

    public static Command disableShooterSubsystems() {
        return new ParallelCommandGroup(
                shooterSubsystem.getInstance().disableSystem(),
                turretSubsystem.getInstance().disableSystem(),
                intakeSubsystem.getInstance().setPowerCommand(0),
                transferSubsystem.getInstance().setPowerCommand(0)
        );
    }
}
