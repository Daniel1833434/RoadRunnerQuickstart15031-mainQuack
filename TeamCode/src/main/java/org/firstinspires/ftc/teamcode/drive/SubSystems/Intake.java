package org.firstinspires.ftc.teamcode.drive.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public DcMotor Intake;
    public DcMotor Intake2;
    public Servo IntakeServo;
    public enum IntakeState{
        NotIntaking,
        Intaking
    }
    public IntakeState currentIntakeState;
    public void Init(){
        Intake  = hardwareMap.get(DcMotor.class,"intake");
        Intake2 = hardwareMap.get(DcMotor.class,"intakeWheel");
        IntakeServo = hardwareMap.get(Servo.class,"IntakeServo");
        currentIntakeState=IntakeState.NotIntaking;
    }

    public void StartIntaking(){
        currentIntakeState=IntakeState.Intaking;
        Intake.setPower(1);
        Intake2.setPower(1);
        IntakeServo.setPosition(1);
    }
    public void  StopIntaking(){
        currentIntakeState=IntakeState.NotIntaking;
        Intake.setPower(0);
        Intake2.setPower(0);
        IntakeServo.setPosition(0);
    }

}
