package org.firstinspires.ftc.teamcode.drive.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {

    Intake intake = new Intake();
    Lift lift = new Lift();
    public Lift.LiftState currentLiftPose;
    public Intake.IntakeState currentIntakeState;

    public SampleMecanumDrive drive;
    public Servo DumpServo;
    public enum DumpServoState{
        IDLE,//Normal position
        Scoring// Position to score
    }
    public DumpServoState DumpState;
    public void InitRobot(Pose2d Startpose){

        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set inital pose
        drive.setPoseEstimate(Startpose);
        lift.Init();
        intake.Init();

        currentIntakeState = intake.currentIntakeState;
        currentLiftPose = lift.currentLiftState;
        DumpServo = hardwareMap.get(Servo.class,"DumpServo");
        DumpState = DumpServoState.IDLE;

    }
    public void Intake(){
        currentIntakeState = intake.currentIntakeState;
        intake.StartIntaking();
    }
    public void StopIntake(){
        currentIntakeState = intake.currentIntakeState;
        intake.StopIntaking();
    }

    public void MoveLift(Lift.LiftState liftState){
        lift.STARTlIFT(liftState);
        currentLiftPose = lift.currentLiftState;
    }
    public void MoveDumpServo(DumpServoState state){
        switch (state){
            case IDLE:
                DumpServo.setPosition(0);
                DumpState = DumpServoState.IDLE;
                break;
            case Scoring:
                DumpServo.setPosition(1);
                DumpState = DumpServoState.Scoring;
                break;
        }

    }
}
