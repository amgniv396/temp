package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class shooterSubSystem extends SubsystemBase {
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
    private double lastVelocity = 0.0;
    private final ElapsedTime timer;
    private static SimplePIDFController pid;
    public static CuttleEncoder encoder;

    private static shooterSubSystem instance;

    public static synchronized shooterSubSystem getInstance() {
        if (instance == null) {
            instance = new shooterSubSystem();
        }
        pid.setPIDF(kp, ki, kd, ks, kv, ka);
        return instance;
    }

    private shooterSubSystem() {
        leftMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
        rightMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 2);

        rightMotor.setDirection(Direction.REVERSE);

        leftMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new SimplePIDFController(kp, ki, kd, ks, kv, ka);

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
}
