package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PidF;

@TeleOp(name = "TeleOp11635")
public class QuackAttackTeleOp11635 extends LinearOpMode{
    PidF pidController = new PidF();

    @Override
    public void runOpMode(){
        pidController.BeforeInitPidf(800,1,1,1,0.05,"LinearSlide",1800,0);//all the vairables for the pidcontroller
        pidController.InitPidf();//Init pidcontroller
        DcMotor Intake = hardwareMap.get(DcMotor.class,"intake");
        DcMotor Intake2 = hardwareMap.get(DcMotor.class,"intakeWheel");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);


        waitForStart();

        while (opModeIsActive()){
            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a){
                Intake.setPower(1);
                Intake2.setPower(1);
            }
            if (gamepad1.b){
                pidController.LoopPidf();//when b pressed hold position at 800 ticks
            }

            Pose2d ResetFieldCentric = new Pose2d(poseEstimate.getX(),poseEstimate.getY(),Math.toRadians(0));
            if (gamepad1.right_stick_button){
                drive.setPoseEstimate(ResetFieldCentric);//Resets the field centric drive when the right stick pressed
            }
            Intake.setPower(0);
            Intake2.setPower(0);
            telemetry.addData("Heading",poseEstimate.getHeading());
            telemetry.addData("X",poseEstimate.getX());
            telemetry.addData("Y",poseEstimate.getY());
            telemetry.update();
        }
   }
}
