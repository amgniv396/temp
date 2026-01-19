package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.PIDController.SimplePIDController;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.mathUtils;

import java.util.function.DoubleSupplier;

public class TurretSubSystem extends SubsystemBase {
    private final CuttleCrServo rightServo;
    private final CuttleCrServo leftServo;
    private SimplePIDController pid;
    private CuttleEncoder encoder;
    private final double MaxRange = 400;
    private final double MinRange = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    private static TurretSubSystem instance;

    public static synchronized TurretSubSystem getInstance() {
        if (instance == null) {
            instance = new TurretSubSystem();
        }
        return instance;
    }

    private TurretSubSystem() {
        rightServo = new CuttleCrServo(JeruRobot.getInstance().controlHub, 1);
        leftServo = new CuttleCrServo(JeruRobot.getInstance().controlHub, 2);

        pid = new SimplePIDController(kp, ki, kd);
        encoder = new CuttleEncoder(JeruRobot.getInstance().controlHub, 0,8192);
        encoder.setPose(0);
    }

    private void setPower(double power) {
        rightServo.setPower(power);
        leftServo.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }

    public Command getToAndHoldPos(DoubleSupplier pos) {
        return new RunCommand(()->setPower(pid.calculate(encoder.getPose(), pos.getAsDouble())));
    }

    public Command targetAtGoal() {
        return getToAndHoldPos(this::getNormalizeTargetAngle);
    }


    public Command targetAtGoalWhileDriving() {
        return getToAndHoldPos(()->(getNormalizeTargetAngle()+computeCompensatedDirection()));
    }

    private double computeCompensatedDirection() {
        Vector2d p = getTargetGoal().minus(JeruRobot.getInstance().localizer.getPositionRR().position);
        Vector2d vel = JeruRobot.getInstance().localizer.getVelocityRR().linearVel;//TODO:check if true

        double A = p.dot(p);
        double B = -2 * p.dot(vel);
        double C = vel.dot(vel) - Math.pow(getExitVal(), 2);

        double disc = Math.pow(B, 2) - 4 * A * C;

        double k1 = (-B + Math.sqrt(disc)) / (2*A);
        double k2 = (-B - Math.sqrt(disc)) / (2*A);

        if (disc < 0 || Math.max(k1, k2) <= 0)
            return Math.toDegrees(JeruRobot.getInstance().localizer.getPositionRR().heading.toDouble());

        return (p.times(Math.max(k1,k2)).minus(vel)).norm();
    }
    private double getExitVal() {
        return 0;
    }

    private Vector2d getTargetGoal() {
        if (JeruRobot.getInstance().allianceColor == AllianceColor.BLUE)
            return new Vector2d(65,65);
        return new Vector2d(-65,65);
    }

    private double getFiledOrientedTargetAngle() {
        return Math.toDegrees(Math.atan2(getTargetGoal().y, JeruRobot.getInstance().localizer.getPositionRR().position.x))
                + 180 - Math.toDegrees(JeruRobot.getInstance().localizer.getPositionRR().heading.toDouble());
    }

    private double getNormalizeTargetAngle() {
        double targetAngle = getFiledOrientedTargetAngle();

        if (targetAngle+360 < MaxRange && mathUtils.is_closer_to(encoder.getPose(), targetAngle, targetAngle+360))
            return targetAngle+360;
        else if (targetAngle-360 > MinRange && mathUtils.is_closer_to(encoder.getPose(), targetAngle, targetAngle-360))
            return targetAngle-360;
        else if (targetAngle < MinRange || targetAngle > MaxRange)
            return  mathUtils.clampInCircle(targetAngle, MinRange, MaxRange);
        return targetAngle;
    }
}
