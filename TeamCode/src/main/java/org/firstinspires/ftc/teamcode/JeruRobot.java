package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.SubSystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.ClawSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;


public class JeruRobot extends Robot {
    public static JeruRobot instance;
    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public VoltageSensor battery;
    public static GoBildaPinpointDriverRR localizer;
    public DriveTrain driveTrain;
    public ClawSubSystem claw;
    public ArmSubSystem arm;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;


    public static synchronized JeruRobot getInstance() {
        if (instance == null) {
            instance = new JeruRobot();
        }
        return instance;
    }

    public void resetRobot() {
        instance = null;
    }

    public void initBarnRobot(OpMode opMode) {
        initBarnRobot(opMode, OpModeType.TELEOP, AllianceColor.RED);
    }
    public void initBarnRobot(OpMode opMode, OpModeType opModeType) {
        initBarnRobot(opMode, opModeType, AllianceColor.RED);
    }
    public void initBarnRobot(OpMode opMode, AllianceColor allianceColor) {
        initBarnRobot(opMode, OpModeType.TELEOP, allianceColor);
    }
    public void initBarnRobot(OpMode opMode, OpModeType opModeType, AllianceColor allianceColor) {
        this.opModeType = opModeType;
        this.allianceColor = allianceColor;

        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        this.controlHub = new CuttleRevHub(hardwareMap, "Control Hub");
        if (opModeType != OpModeType.EXPERIMENTING_NO_EXPANSION) {
            this.expansionHub = new CuttleRevHub(hardwareMap, "Expansion Hub");
        }

        gamepadEx1 = new GamepadEx(opMode.gamepad1);
        gamepadEx2 = new GamepadEx(opMode.gamepad2);

        battery = hardwareMap.voltageSensor.iterator().next();



        //TODO:may need to change name based on your control and expansion hubs name

    }

    public void initBarnRobotSystems() {
        claw = new ClawSubSystem();
        arm = new ArmSubSystem();
        initDriveTrain(gamepadEx1.getLeftX(), gamepadEx1.getLeftY(), gamepadEx1.getRightX());
    }

    private void initDriveTrain(double x, double y, double yaw) {
        driveTrain = new DriveTrain();
        driveTrain.setDefaultCommand(
                instance.driveTrain.fieldOrientedDrive(
                        ()-> Math.pow(x,3),
                        () -> Math.pow(y,3),
                        () -> Math.pow(yaw,3))
        );
    }
}
