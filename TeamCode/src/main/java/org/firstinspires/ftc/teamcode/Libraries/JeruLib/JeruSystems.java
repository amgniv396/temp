package org.firstinspires.ftc.teamcode.Libraries.JeruLib;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class JeruSystems {

    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public VoltageSensor battery;
    public GoBildaPinpointDriverRR localizer;

    private void initDriveTrainDefaultCommand() {
        DriveTrain.getInstance().setDefaultCommand(
                DriveTrain.getInstance().fieldOrientedDriveCommand());
    }
    private void initSystems(OpMode opMode) {
        //TODO:may need to change name based on your control and expansion hubs name
        this.controlHub = new CuttleRevHub(hardwareMap, "Control Hub");
        if (JeruRobot.getInstance().opModeType != OpModeType.EXPERIMENTING_NO_EXPANSION) {
            this.expansionHub = new CuttleRevHub(hardwareMap, "Expansion Hub 2");
        }

        gamepadEx1 = new GamepadEx(opMode.gamepad1);
        gamepadEx2 = new GamepadEx(opMode.gamepad2);

        battery = hardwareMap.voltageSensor.iterator().next();
    }
    private void initLocalize(Pose2d currentPose) {
        localizer = hardwareMap.get(GoBildaPinpointDriverRR.class, "imu");
        localizer.resetPosAndIMU();
        localizer.setOffsets(-99, 9);
        localizer.setEncoderResolution(GoBildaPinpointDriverRR.goBILDA_4_BAR_POD);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        localizer.setPosition(new Pose2d(0, 0, localizer.getPositionRR().heading.toDouble() - Math.toRadians(90)));
//        localizer.setPosition(new Pose2d(currentPose.position, currentPose.heading.toDouble() - Math.toRadians(90)));
    }

    protected void initJeruSystems(OpMode opMode) {
        initSystems(opMode);
        initDriveTrainDefaultCommand();
        initLocalize(new Pose2d(0,0,0));
    }
}
