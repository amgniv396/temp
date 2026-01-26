package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.intakeCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.shootCommandGroup;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.DriveActionCommand;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.intakeSubsystem;

//TODO:rot
@Autonomous
public class redClose extends CommandOpMode {
    static JeruRobot robotInstance;
    public static final Pose2d scorePose = new Pose2d(-58.5, -54.5, Math.toRadians(240));
    final Pose2d intakePose = new Pose2d(-24, -9, Math.toRadians(180));

    boolean flag = false;

    @Override
    public void initialize() {
        robotInstance = JeruRobot.getInstance();
        Pose2d startPos = new Pose2d(-60,60,Math.toRadians(60));
        MecanumDrive drive = new MecanumDrive(JeruRobot.getInstance().hardwareMap, startPos);

        TrajectoryActionBuilder driveToIntakeFirst = drive.actionBuilder(startPos)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-40, 40,90), Math.toRadians(90),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.7),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.4, MecanumDrive.PARAMS.maxProfileAccel * 0.7));



        new SequentialCommandGroup(
                //preload
                shootCommandGroup.shootAll(),

                //first
                new DriveActionCommand(driveToIntakeFirst).alongWith(
                        intakeCommandGroup.intakeCommand())

        ).schedule();
    }


}
