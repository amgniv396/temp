
package org.firstinspires.ftc.teamcode.SubSystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Libraries.JeruLib.JeruRobot;

import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {
    private static DriveTrain instance;
    static double[][] transformationMatrix = {
            {1, 1, 1}, //frontLeft
            {-1, 1, 1}, //backLeft
            {-1, 1, -1}, //frontRight
            {1, 1, -1} //backRight
    };

    private final DcMotorEx motorFR;
    private final DcMotorEx motorFL;
    private final DcMotorEx motorBL;
    private final DcMotorEx motorBR;

    private final double slowmodeMultiplayer = 0.3;
    private final double slowmodeYawMultiplayer = 0.3;


    public static synchronized DriveTrain getInstance() {
        if (instance == null) {
            instance = new DriveTrain();
        }
        return instance;
    }

    private DriveTrain() {
        super(); //register this subsystem, in order to schedule default command later on.

        motorFL = JeruRobot.getInstance().hardwareMap.get(DcMotorEx.class, "FL");
        motorBL = JeruRobot.getInstance().hardwareMap.get(DcMotorEx.class, "BL");
        motorFR = JeruRobot.getInstance().hardwareMap.get(DcMotorEx.class, "FR");
        motorBR = JeruRobot.getInstance().hardwareMap.get(DcMotorEx.class, "BR");

        //TODO: reverse motors
    }

    public void setYaw(double lastAngle){
        //mmRobot.mmSystems.imu.setYaw(lastAngle);//TODO:insert robot last yaw from autonomous
    }


    private double[] joystickToPower(double x, double y, double yaw) {

        //v = (x, y, yaw)^t (3x1)
        RealVector joystickVector = MatrixUtils.createRealVector(new double[] {
                x,
                y,
                yaw
        });

        RealMatrix matrixT = MatrixUtils.createRealMatrix(transformationMatrix); //4x3

        //calculation of the power needed by T constants
        RealVector powerVector = matrixT.operate(joystickVector); //p = Tv

        double[] powerArray = powerVector.toArray(); //4x1

        //normalize the array
        for(int i = 0; i < powerArray.length; i++) {
            powerArray[i] = powerArray[i] / Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        }

        return powerArray;

    }

    private void setMotorPower(double[] power) {
        motorFL.setPower(power[0]);
        motorBL.setPower(power[1]);
        motorFR.setPower(power[2]);
        motorBR.setPower(power[3]);
//        updateTelemetry(power);
    }
    public void drive(double x, double y, double yaw) {
        setMotorPower(joystickToPower(x, y, yaw));
    }

    public void resetYaw() {
        JeruRobot.getInstance().localizer.resetYaw();
    }
    public Command resetYawCommand(){
        return new InstantCommand(this::resetYaw);
    }
    private void fieldOrientedDrive(double x, double y, double yaw) {
        Vector2d joystickDirection = new Vector2d(x, y);
        Vector2d fieldOrientedVector = joystickDirection.rotateBy(
                JeruRobot.getInstance().startAngle+JeruRobot.getInstance().localizer.getYawScalar());//TODO:check
        drive(fieldOrientedVector.getX(), fieldOrientedVector.getY(), yaw);
    }

    public Command fieldOrientedDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier yaw) {
        return new RunCommand(() -> fieldOrientedDrive(x.getAsDouble(),y.getAsDouble(),yaw.getAsDouble()), this);
    }
    public Command fieldOrientedDriveCommand() {
        return fieldOrientedDriveCommand(
                () -> Math.pow(JeruRobot.getInstance().gamepadEx1.getLeftX(), 3),
                () -> Math.pow(JeruRobot.getInstance().gamepadEx1.getLeftY(), 3),
                () -> Math.pow(JeruRobot.getInstance().gamepadEx1.getRightX(), 3)
        );
    }
    public Command slowmodeFieldOrientedDriveCommand() {
        return slowmodeFieldOrientedDriveCommand(
                () -> JeruRobot.getInstance().gamepadEx1.getLeftX(),
                () -> JeruRobot.getInstance().gamepadEx1.getLeftY(),
                () -> JeruRobot.getInstance().gamepadEx1.getRightX()
        );
    }
    public Command slowmodeFieldOrientedDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier yaw) {
        return fieldOrientedDriveCommand(
                () -> Math.pow(x.getAsDouble(),5) * slowmodeMultiplayer,
                () -> Math.pow(y.getAsDouble(),5) * slowmodeMultiplayer,
                () -> Math.pow(yaw.getAsDouble(),5) * slowmodeYawMultiplayer
        );
    }


    public void updateTelemetry(double[] power) {
        FtcDashboard.getInstance().getTelemetry().addData("frontLeft", power[0]);
        FtcDashboard.getInstance().getTelemetry().addData("backLeft", power[1]);
        FtcDashboard.getInstance().getTelemetry().addData("frontRight", power[2]);
        FtcDashboard.getInstance().getTelemetry().addData("backRight", power[3]);
        FtcDashboard.getInstance().getTelemetry().update();
    }

}
