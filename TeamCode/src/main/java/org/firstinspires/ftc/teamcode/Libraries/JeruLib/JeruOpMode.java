package org.firstinspires.ftc.teamcode.Libraries.JeruLib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Libraries.JeruLib.Utils.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.Drawing;

public abstract class JeruOpMode extends CommandOpMode {
    @Override
    public abstract void initialize();

    @Override
    public void run() {
        super.run();
        if (JeruRobot.getInstance().opModeType!= OpModeType.EXPERIMENTING_NO_EXPANSION){
            JeruRobot.getInstance().expansionHub.pullBulkData();
        }
        JeruRobot.getInstance().localizer.update();
        telemetry.addData("a", Math.toDegrees(JeruRobot.getInstance().localizer.getPositionRR().heading.toDouble()));

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), JeruRobot.getInstance().localizer.getPositionRR());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
    @Override
    public void end() {
        JeruRobot.getInstance().resetRobot();
    }
}
