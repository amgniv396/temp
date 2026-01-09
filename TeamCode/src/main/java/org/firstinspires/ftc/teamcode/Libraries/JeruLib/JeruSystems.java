package org.firstinspires.ftc.teamcode.Libraries.JeruLib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

import java.util.ArrayList;

public class JeruSystems {

    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public VoltageSensor battery;

    protected void initDriveTrainDefaultCommand(double x, double y, double yaw) {
        DriveTrain.getInstance().setDefaultCommand(
                DriveTrain.getInstance().fieldOrientedDriveCommand(
                        ()-> Math.pow(x,3),
                        () -> Math.pow(y,3),
                        () -> Math.pow(yaw,3)
        ));
    }
    protected void initSystems(OpMode opMode) {
        //TODO:may need to change name based on your control and expansion hubs name
        this.controlHub = new CuttleRevHub(JeruRobot.getInstance().hardwareMap, "Control Hub");
        if (JeruRobot.getInstance().opModeType != OpModeType.EXPERIMENTING_NO_EXPANSION) {
            this.expansionHub = new CuttleRevHub(JeruRobot.getInstance().hardwareMap, "Expansion Hub");
        }

        gamepadEx1 = new GamepadEx(opMode.gamepad1);
        gamepadEx2 = new GamepadEx(opMode.gamepad2);

        battery = JeruRobot.getInstance().hardwareMap.voltageSensor.iterator().next();
    }
    protected void initJeruRobot(OpMode opMode) {
        initSystems(opMode);
        initDriveTrainDefaultCommand(gamepadEx1.getLeftX(), gamepadEx1.getLeftY(), gamepadEx1.getRightX());
    }
}
