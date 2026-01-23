package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.PIDController.SimplePIDFController;

@Config
public class shooterSubSystem extends SubsystemBase {
    private final CuttleMotor leftMotor;
    private final CuttleMotor rightMotor;
    public static int Fast = 2000;
    public static int Mid = 1500;
    public static int Slow = 1000;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;

    private SimplePIDFController pid;

    private static shooterSubSystem instance;

    public static synchronized shooterSubSystem getInstance() {
        if (instance == null) {
            instance = new shooterSubSystem();
        }
        return instance;
    }

    private shooterSubSystem() {
        leftMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 3);
        rightMotor = new CuttleMotor(JeruRobot.getInstance().controlHub, 2);

        rightMotor.setDirection(Direction.REVERSE);

        leftMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new SimplePIDFController(kp, ki, kd, ks, kv, ka);
    }

    private void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power),this);
    }
}
