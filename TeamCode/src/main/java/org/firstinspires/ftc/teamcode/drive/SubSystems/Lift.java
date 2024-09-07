package org.firstinspires.ftc.teamcode.drive.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.PID;

public class Lift {

    public enum LiftState {
        Down,
        Middle,
        Up,
        Stop
    }

    PID Down_LiftpidController = new PID();
    PID Middle_LiftpidController = new PID();
    PID Up_LiftpidController = new PID();

    PID Down_LiftpidController2 = new PID();
    PID Middle_LiftpidController2 = new PID();
    PID Up_LiftpidController2 = new PID();

    public LiftState currentLiftTarget;
    public LiftState currentLiftPose;
    public DcMotorEx LiftMotor;
    public DcMotorEx LiftMotor2;
    public void Init(){
        Down_LiftpidController.BeforeInitPid(0,1,1,1,"Lift",1800,0);//all the vairables for the pidcontroller
        Down_LiftpidController.InitPid();//Init pidcontroller
        Middle_LiftpidController.BeforeInitPid(3000,1,1,1,"Lift",1800,0);//all the vairables for the pidcontroller
        Middle_LiftpidController.InitPid();//Init pidcontroller
        Up_LiftpidController.BeforeInitPid(6000,1,1,1,"Lift",1800,0);//all the vairables for the pidcontroller
        Up_LiftpidController.InitPid();//Init pidcontroller

        Down_LiftpidController2.BeforeInitPid(0,1,1,1,"Lift2",1800,0);//all the vairables for the pidcontroller
        Down_LiftpidController2.InitPid();//Init pidcontroller
        Middle_LiftpidController2.BeforeInitPid(3000,1,1,1,"Lift2",1800,0);//all the vairables for the pidcontroller
        Middle_LiftpidController2.InitPid();//Init pidcontroller
        Up_LiftpidController2.BeforeInitPid(6000,1,1,1,"Lift2",1800,0);//all the vairables for the pidcontroller
        Up_LiftpidController2.InitPid();//Init pidcontroller
        currentLiftTarget = LiftState.Down;
        currentLiftPose = LiftState.Down;

        LiftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        LiftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMotor2");

    }
    public void STARTlIFT(LiftState state){
        switch (state){
            case Down:
                Down_LiftpidController.LoopPid();
                Down_LiftpidController2.LoopPid();
                currentLiftTarget = LiftState.Down;
                if(LiftMotor.getCurrentPosition() <75){
                    currentLiftPose =LiftState.Down;
                }
                break;

            case Middle:
                Middle_LiftpidController.LoopPid();
                Middle_LiftpidController2.LoopPid();
                currentLiftTarget = LiftState.Middle;
                if(LiftMotor.getCurrentPosition() <3075 || LiftMotor.getCurrentPosition()>2925){
                    currentLiftPose =LiftState.Middle;
                }
                break;

            case Up:
                Up_LiftpidController.LoopPid();
                Up_LiftpidController2.LoopPid();
                currentLiftTarget = LiftState.Up;
                if(LiftMotor.getCurrentPosition() <6075 || LiftMotor.getCurrentPosition()>5925){
                    currentLiftPose =LiftState.Up;
                }
                break;

            case Stop:
                LiftMotor.setPower(0);
                LiftMotor2.setPower(0);

        }
    }
}
