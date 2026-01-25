package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.PIDController.SimplePIDFController;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;

@Config
public class shooterSubsystem extends SubsystemBase {
    private final CuttleMotor leftMotor;
    private final CuttleMotor rightMotor;
    public static double Fast = 80;
    public static double Mid = 60;
    public static double Slow = 40;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;
    public static double tolerance = 0;
    private double lastVelocity = 0.0;
    private final ElapsedTime timer;
    private static SimplePIDFController pid;
    public static CuttleEncoder encoder;

    private static shooterSubsystem instance;

    public static synchronized shooterSubsystem getInstance() {
        if (instance == null) {
            instance = new shooterSubsystem();
        }
        pid.setPIDF(kp, ki, kd, ks, kv, ka);
        return instance;
    }

    private shooterSubsystem() {
        leftMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
        rightMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 2);

        rightMotor.setDirection(Direction.REVERSE);

        leftMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new SimplePIDFController(kp, ki, kd, ks, kv, ka);
        pid.setTolerance(tolerance);

        encoder = new CuttleEncoder(JeruRobot.getInstance().controlHub, 3, 28);

        timer = new ElapsedTime();
    }

    private void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }

    public Command setRPMCommand(double rpm) {
        return new RunCommand(() ->{
            double currentVelocity = encoder.getVelocity();
            double dt = timer.seconds();

            double accel = 0.0;
            if (dt > 0 && dt < 0.5) {
                accel = (currentVelocity - lastVelocity) / dt;
            }

            setPower(pid.calculate(currentVelocity, rpm, currentVelocity, accel));

            lastVelocity = currentVelocity;
            timer.reset();
        });
    }

    public Command setDistanceBasedRPM() {
        return setRPMCommand(getDistanceBasedVal());
    }

    private double getDistanceBasedAlliance() {
        if (JeruRobot.getInstance().allianceColor == AllianceColor.RED)
            return JeruRobot.getInstance().localizer.getPositionRR().position.minus(new Vector2d(65,65)).norm();
        return JeruRobot.getInstance().localizer.getPositionRR().position.minus(new Vector2d(-65,65)).norm();
    }

    private double getDistanceBasedVal() {
        double dist = getDistanceBasedAlliance();
        if (dist < 50)
            return Slow;
        else if (dist < 100)
            return Mid;
        return Fast;
    }

    public Boolean atSetPoint() {
        return pid.atSetpoint();
    }
    public Command disableSystem() {
        return new InstantCommand(()->{},this);
    }
}
