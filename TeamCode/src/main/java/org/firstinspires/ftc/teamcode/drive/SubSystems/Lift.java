package org.firstinspires.ftc.teamcode.drive.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.PID;

public class Lift {

    public enum LiftState {
        Down,
        Middle,
        Up
    }

    PID Down_LiftpidController = new PID();
    PID Middle_LiftpidController = new PID();
    PID Up_LiftpidController = new PID();

    PID Down_LiftpidController2 = new PID();
    PID Middle_LiftpidController2 = new PID();
    PID Up_LiftpidController2 = new PID();

    LiftState currentLiftState;
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
        currentLiftState = LiftState.Down;


    }
    public void STARTlIFT(LiftState state){
        switch (state){
            case Down:
                Down_LiftpidController.LoopPid();
                Down_LiftpidController2.LoopPid();
                currentLiftState = LiftState.Down;
                break;

            case Middle:
                Middle_LiftpidController.LoopPid();
                Middle_LiftpidController2.LoopPid();
                currentLiftState = LiftState.Middle;
                break;

            case Up:
                Up_LiftpidController.LoopPid();
                Up_LiftpidController2.LoopPid();
                currentLiftState = LiftState.Up;
                break;


        }
    }
}
