package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.drive.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PidF;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import com.sfdev.assembly.state.*;

@TeleOp(name = "Best TeleOp11635")
public class QuackAttackTeleOp11635 extends LinearOpMode{

    public enum States{
        Idle, // pid for moving the lift is down and not intaking and dump reseted can Intake and can MoveLift
        Intaking, // Start Intaking and pid for moving the lift down cant Move Lift(rt)
        LiftingMid, // moving the lift to Middle and cant intake but can score(a)
        LiftingUp, // moving the lift to Up and cant intake but can score(b)
        Scroring //Dumping waiting a sec and going back to idle cant move lift or intake(y)

    }
    Robot robot = new Robot();
    @Override
    public void runOpMode(){
        //Init//
        robot.InitRobot(PoseStorage.currentPose);
        SampleMecanumDrive drive = robot.drive;
        //Init finished//

        StateMachine machine = new StateMachineBuilder()
                .state(States.Idle)
                .onEnter(()->robot.StopIntake())
                .onEnter(()->robot.MoveDumpServo(Robot.DumpServoState.IDLE))
                .loop(()->robot.MoveLift(Lift.LiftState.Down))
                .transition(()->gamepad2.right_trigger>0.4,States.Intaking)
                .transition(()->gamepad2.a,States.LiftingMid)
                .transition(()->gamepad2.b,States.LiftingUp)

                .state(States.Intaking)
                .onEnter(()->robot.Intake())
                .onEnter(()->robot.MoveDumpServo(Robot.DumpServoState.IDLE))
                .loop(()->robot.MoveLift(Lift.LiftState.Down))
                .transition(()->gamepad2.right_trigger<0.4,States.Idle)

                .state(States.LiftingMid)
                .loop(()->robot.MoveLift(Lift.LiftState.Middle))
                .transition(()-> gamepad2.y,States.Scroring)
                .transition(()->gamepad2.a,States.Idle)
                .transition(()->gamepad2.b,States.LiftingUp)

                .state(States.LiftingUp)
                .loop(()->robot.MoveLift(Lift.LiftState.Up))
                .transition(()-> gamepad2.y,States.Scroring)
                .transition(()->gamepad2.b,States.Idle)
                .transition(()->gamepad2.a,States.LiftingMid)

                .state(States.Scroring)
                .onEnter(()->robot.MoveLift(Lift.LiftState.Stop))
                .onEnter(()->robot.MoveDumpServo(Robot.DumpServoState.Scoring))
                .transitionTimed(1,States.Idle)

                .build();

        waitForStart();

        machine.start();
        while (opModeIsActive()){

            machine.update();
            drive.updatePoseEstimate();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            gamepad1.right_stick_x
                    )
            );

            Pose2d ResetFieldCentric = new Pose2d(poseEstimate.getX(),poseEstimate.getY(),Math.toRadians(0));
            if (gamepad1.right_stick_button){
                drive.setPoseEstimate(ResetFieldCentric);//Resets the field centric drive when the right stick pressed
            }

            //Stop Intake and lift
            robot.StopIntake();
            robot.MoveLift(Lift.LiftState.Stop);

            telemetry.addData("Heading",poseEstimate.getHeading());
            telemetry.addData("X",poseEstimate.getX());
            telemetry.addData("Y",poseEstimate.getY());
            telemetry.addData("IntakeState",robot.currentIntakeState);
            telemetry.addData("LiftState",robot.currentLiftPose);
            telemetry.addData("DumpState",robot.DumpState);
            telemetry.update();
        }
   }
}
