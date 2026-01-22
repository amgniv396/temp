package org.firstinspires.ftc.teamcode.Libraries.JeruLib;

import static com.qualcomm.robotcore.util.RobotLog.a;
import static com.qualcomm.robotcore.util.RobotLog.d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;

import java.util.ArrayList;
import java.util.List;


public class JeruRobot extends JeruSystems {
    private static JeruRobot instance;
    public OpModeType opModeType;
    public AllianceColor allianceColor;
    public int startAngle;

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

    private void initJeruBasics(OpMode opMode, OpModeType opModeType, AllianceColor allianceColor, int startAngle) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        this.opModeType = opModeType;
        this.allianceColor = allianceColor;
        this.startAngle = startAngle;

        initJeruSystems(opMode);
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
            JeruRobot.getInstance().initJeruBasics(opMode, opModeType, allianceColor, startAngle);
        }
    }
}
