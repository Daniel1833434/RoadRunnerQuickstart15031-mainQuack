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

@TeleOp(name = "Best TeleOp11635")
public class QuackAttackTeleOp11635 extends LinearOpMode{

    Robot robot = new Robot();
    @Override
    public void runOpMode(){
        //Init//
        robot.InitRobot(PoseStorage.currentPose);
        SampleMecanumDrive drive = robot.drive;
        //Init finished//

        waitForStart();

        while (opModeIsActive()){
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

            if (gamepad2.right_trigger>0.4){
                Intake.setPower(1);
                Intake2.setPower(1);
            }else if(gamepad2.left_trigger>0.4){
                Intake.setPower(-1);
                Intake2.setPower(-1);
            }

            if (gamepad2.right_bumper){
                RightLift.setPower(-1);
                //LeftLift.setPower(1);
                //pidController.LoopPidf();//when b pressed hold position at 800 ticks
            }else if(gamepad2.left_bumper){
                RightLift.setPower(1);
                //LeftLift.setPower(-1);
            }

            if(gamepad2.b){
                //BucketLeft.setPosition(1);//change
                //BucketRight.setPosition(-1);//change
            }else if(gamepad2.x){
                //BucketLeft.setPosition(-1);//change
                //BucketRight.setPosition(1);//change
            }
            if(gamepad2.a){
                //OpenBucket.setPosition(-1);//change if needed
            }else if(gamepad2.y){
                //OpenBucket.setPosition(0);//change if needed
            }


            Pose2d ResetFieldCentric = new Pose2d(poseEstimate.getX(),poseEstimate.getY(),Math.toRadians(0));
            if (gamepad1.right_stick_button){
                drive.setPoseEstimate(ResetFieldCentric);//Resets the field centric drive when the right stick pressed
            }

            Intake.setPower(0);
            Intake2.setPower(0);
            RightLift.setPower(0);
            //LeftLift.setPower(0);

            telemetry.addData("Heading",poseEstimate.getHeading());
            telemetry.addData("X",poseEstimate.getX());
            telemetry.addData("Y",poseEstimate.getY());
            telemetry.update();
        }
   }
}
