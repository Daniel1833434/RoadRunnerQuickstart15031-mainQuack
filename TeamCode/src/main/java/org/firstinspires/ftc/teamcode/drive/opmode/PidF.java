package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTooManySequentialErrorsException;

public class PidF {
    private PIDController controller;
    private double p,i,d,f;
    private int Target;
    private double ticks_in_degree;
    private DcMotorEx Motor;
    private String MotorName;
    private double ZeroOffset;//can be caculated by looking at the encoder value when the arm is in its 0 position

    public void BeforeInitPidf(int target,double kp, double ki, double kd, double kf,String motorName,int ticksperrot,double ZeroOffset){
        p=kp;
        i=ki;
        d=kd;
        f=kf;
        Target=target;//in ticks
        MotorName = motorName;
        ticks_in_degree = ticksperrot / 360;//ticksperrot is the total ticks in rot and 360 is degrees for full rot
        this.ZeroOffset = ZeroOffset;

    }
    public void InitPidf(){
        controller = new PIDController(p,i,d);
        Motor = hardwareMap.get(DcMotorEx.class, MotorName);
        //controller.setPID(p,i,d); if nedeed delete the "//"

    }
    public void LoopPidf(){
        int armPos = Motor.getCurrentPosition();
        double pid = controller.calculate(armPos, Target);
        double ff = Math.cos(Math.toRadians(Target/ticks_in_degree+ZeroOffset))*f;

        double power = pid + ff ;
        Motor.setPower(power);


    }
}
