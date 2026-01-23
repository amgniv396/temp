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
        //get sensors data
        JeruRobot.getInstance().controlHub.pullBulkData();
        if (JeruRobot.getInstance().opModeType != OpModeType.EXPERIMENTING_NO_EXPANSION){
            JeruRobot.getInstance().expansionHub.pullBulkData();
        }
        JeruRobot.getInstance().localizer.update();

        //fieldOverlay dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), JeruRobot.getInstance().localizer.getPositionRR());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        //update outputs
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
    @Override
    public void end() {
        JeruRobot.getInstance().resetRobot();
    }
}
