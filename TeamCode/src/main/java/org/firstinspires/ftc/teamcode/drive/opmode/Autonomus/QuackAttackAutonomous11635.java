package org.firstinspires.ftc.teamcode.drive.opmode.Autonomus;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto 1")
public class QuackAttackAutonomous11635 extends LinearOpMode{
    @Override
    public void runOpMode(){
        int counter =0;
        DcMotor Intake = hardwareMap.get(DcMotor.class,"IntakeMotor");

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d StartingPose = new Pose2d(-40,-64,Math.toRadians(0));//The starting pose- dont change the angle(0)!!!

        drive.setPoseEstimate(StartingPose);//Starting pose

        // Making all the paths
        Trajectory FirstDrive = drive.trajectoryBuilder(StartingPose)//Starting pose
                .splineTo(new Vector2d(-55,-35),Math.toRadians(90))//target psoition
                .build();


        Trajectory SecondDrive = drive.trajectoryBuilder(FirstDrive.end())//last position
                .forward(5,SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))// Makes the drive slower
                .addTemporalMarker(0, () -> {
                    // Run action at 0 second into the path
                    Intake.setPower(1);
                })
                .build();

        TrajectorySequence ts = drive.trajectorySequenceBuilder(SecondDrive.end())
                .waitSeconds(3) // Waits 3 seconds
                .addTemporalMarker(3, () -> {
                    // Run action at 3 second into the path
                    Intake.setPower(0);
                })
                .build();

        Trajectory ThirdDrive = drive.trajectoryBuilder(SecondDrive.end())//last position
                .lineToLinearHeading(new Pose2d(48,-35,Math.toRadians(270)))
                .build();

        Trajectory FourthDrive = drive.trajectoryBuilder(ThirdDrive.end())//last position
                .strafeRight(25)
                .build();

        Trajectory FifthDrive = drive.trajectoryBuilder(FourthDrive.end())
                .forward(10)
                .build();
        //The actual run

        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectory(FirstDrive);//FirstDrive and turn on intake motor after 1 second
        drive.followTrajectory(SecondDrive);//SecondDrive is slower than other to pick pixels
        drive.followTrajectorySequence(ts);//waits for 3 seconds and stops intake motor
        drive.followTrajectory(ThirdDrive);//ThirdDrive
        drive.followTrajectory(FourthDrive);//FourthDrive
        drive.followTrajectory(FifthDrive);//FifthDrive
        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.getPoseEstimate();//Transfer the pose to TeleOp;


    }
}
