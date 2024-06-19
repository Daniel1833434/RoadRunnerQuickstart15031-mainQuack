package org.firstinspires.ftc.teamcode.drive.opmode.Autonomus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.PoseStoragePedroPathing;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.opencv.core.Mat;

@Autonomous(name = "PedroPathingAuto")
public class PedroPathingAutonomous extends OpMode{
    private Follower follower;
    int counter=0;
    public PathChain Path1;
    public PathChain Path2;
    public DcMotor Intake;
    public DcMotor Intake2;
    public void init(){
        Intake = hardwareMap.get(DcMotor.class,"intake");
        Intake2 = hardwareMap.get(DcMotor.class,"intakeWheel");
        Pose startingpose = new Pose(10,40, Math.toRadians(0));//dont chhange the heading from zero
        follower = new Follower(hardwareMap,true);
        follower.setStartingPose(startingpose);
         Path1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(10,40, Point.CARTESIAN), new Point(25,22, Point.CARTESIAN), new Point(40,5, Point.CARTESIAN)))
                .build();
         Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(40,5, Point.CARTESIAN), new Point(25,22, Point.CARTESIAN), new Point(10,40, Point.CARTESIAN)))
                .build();


    }
    public void loop(){
        if(!follower.isBusy()){
            if(counter==0){
                Intake.setPower(1);
                Intake2.setPower(1);
                follower.followPath(Path1);
                counter++;

            }else if(counter==1){
                Intake2.setPower(0);
                Intake.setPower(0);
                follower.followPath(Path2);
                counter++;
            }else if(counter==2){
                PoseStoragePedroPathing.currentpose = follower.getPose();
            }
        }

    }

}


