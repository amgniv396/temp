
package org.firstinspires.ftc.teamcode.SubSystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.BarnRobot;

public class DriveTrain extends SubsystemBase {

    final double[][] transformationMatrix = {
            {1, 1, 1}, //frontLeft
            {-1, 1, 1}, //backLeft
            {-1, 1, -1}, //frontRight
            {1, 1, -1} //backRight
    };

    BarnRobot robotInstance = BarnRobot.getInstance();

    private final Motor motorFR;
    private final Motor motorFL;
    private final Motor motorBL;
    private final Motor motorBR;

    public DriveTrain() {
        super(); //register this subsystem, in order to schedule default command later on.

        motorFL = hardwareMap.get(Motor.class, "FL");
        motorBL = hardwareMap.get(Motor.class, "BL");
        motorFR = hardwareMap.get(Motor.class, "FR");
        motorBR = hardwareMap.get(Motor.class, "BR");

        //TODO: reverse motors
    }

    public DriveTrain(double lastAngle){
        this();
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
        motorFL.set(power[0]);
        motorBL.set(power[1]);
        motorFR.set(power[2]);
        motorBR.set(power[3]);
//        updateTelemetry(power);
    }
    public void drive(double x, double y, double yaw) {
        setMotorPower(joystickToPower(x, y, yaw));
    }


    public void fieldOrientedDrive(double x, double y, double yaw) {
        Vector2d joystickDirection = new Vector2d(x, y);
        Vector2d fieldOrientedVector = joystickDirection.rotateBy(0.0);//TODO:insert robot yaw
        drive(fieldOrientedVector.getX(), fieldOrientedVector.getY(), yaw);
    }


//    public void updateTelemetry(double[] power) {
//        FtcDashboard.getInstance().getTelemetry().addData("frontLeft", power[0]);
//        FtcDashboard.getInstance().getTelemetry().addData("backLeft", power[1]);
//        FtcDashboard.getInstance().getTelemetry().addData("frontRight", power[2]);
//        FtcDashboard.getInstance().getTelemetry().addData("backRight", power[3]);
//        FtcDashboard.getInstance().getTelemetry().update();
//    }

}
