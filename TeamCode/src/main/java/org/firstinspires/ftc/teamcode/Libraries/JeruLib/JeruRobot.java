package org.firstinspires.ftc.teamcode.Libraries.JeruLib;

import static com.qualcomm.robotcore.util.RobotLog.a;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.ArrayList;


public class JeruRobot extends JeruSystems {
    private static JeruRobot instance;
    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public int startAngle;
    public static GoBildaPinpointDriverRR localizer;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    private JeruRobot(){}

    public static synchronized JeruRobot getInstance() {
        if (instance == null) {
            instance = new JeruRobot();
        }
        return instance;
    }

    public void resetRobot() {
        instance = null;
    }

    private void initJeruRobot(OpMode opMode, OpModeType opModeType, AllianceColor allianceColor, int startAngle) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        this.opModeType = opModeType;
        this.allianceColor = allianceColor;
        this.startAngle = startAngle;

        initJeruRobot(opMode);
    }

    public Builder initJeruRobot() {
        return new Builder();
    }

    public static class Builder {
        private OpModeType opModeType = OpModeType.TELEOP;
        private AllianceColor allianceColor = AllianceColor.RED;
        private int startAngle = 0;

        public Builder opModeType(OpModeType opModeType) {
            this.opModeType = opModeType;
            return this;
        }
        public Builder allianceColor(AllianceColor allianceColor) {
            this.allianceColor = allianceColor;
            return this;
        }
        public Builder angle(int startAngle) {
            this.startAngle = startAngle;
            return this;
        }

        public void build(OpMode opMode) {
            JeruRobot.getInstance().initJeruRobot(opMode, opModeType, allianceColor, startAngle);
        }
    }
}
