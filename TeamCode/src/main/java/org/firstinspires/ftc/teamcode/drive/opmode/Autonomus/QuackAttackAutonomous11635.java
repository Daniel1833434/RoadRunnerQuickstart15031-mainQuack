package org.firstinspires.ftc.teamcode.drive.opmode.Autonomus;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.drive.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.PID;
import org.firstinspires.ftc.teamcode.drive.opmode.PidF;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import com.sfdev.assembly.state.*;

@Autonomous(name = "Auto1")
public class QuackAttackAutonomous11635 extends LinearOpMode {
    //TODO CHANGE THE STATES AS MUCH AS YOU NEED
    // This enum defines our "state"
    enum State {
        TRAJECTORY_1,   // go to pixel stack
        TRAJECTORY_2,   // Then start intake and go forward
        WAIT_1,         // Then we want to wait for the intake to intake and then stop it
        TRAJECTORY_3,   //  go to the backdrop while lifting the lift, when we are in the backdrop we want to score
        TRAJECTORY4,    // Then we're gonna strafeRight and return the dump servo to his normal position
        TRAJECTORY5,    // we're gonna park in the parking zone
        IDLE,            // Our bot will enter the IDLE until he starts
        STOP             //stop and restart lift
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // TODO Define our start pose
    Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(0));

    Robot robot = new Robot();
    ElapsedTime waitTimer1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.InitRobot(startPose);
        SampleMecanumDrive drive = robot.drive;
        //TODO:MAKE SURE TO MAKE LIFT DOWN AND INTAKE UP IN THE END OF THE AUTO

        //TODO Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-55, -35), Math.toRadians(90))
                .build();

        // Second trajectory
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(5)
                .build();


        //double turnAngle1 = Math.toRadians(-270);
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        //Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));


        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(48, -35,Math.toRadians(270)))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .strafeRight(25)
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .forward(10)
                .build();

        //Define The StateMachine
        StateMachine machine = new StateMachineBuilder()
                .state(State.IDLE)
                .transition(()->isStarted(),State.TRAJECTORY_1)

                .state(State.TRAJECTORY_1)
                .onEnter( ()->drive.followTrajectoryAsync(trajectory1))
                .transition(()->!drive.isBusy(),State.TRAJECTORY_2)

                .state(State.TRAJECTORY_2)
                .onEnter(()->robot.intake.StartIntaking())
                .onEnter(()->drive.followTrajectoryAsync(trajectory2))
                .transition(()->!drive.isBusy(),State.WAIT_1)

                .state(State.WAIT_1)
                .transitionTimed(2,State.TRAJECTORY_3)

                .state(State.TRAJECTORY_3)
                .onEnter(()->robot.intake.StopIntaking())
                .onEnter( ()->drive.followTrajectoryAsync(trajectory3))
                .loop(()->robot.lift.STARTlIFT(Lift.LiftState.Up))
                .onExit(()->robot.MoveDumpServo(Robot.DumpServoState.Scoring))
                .transition(()->!drive.isBusy(),State.TRAJECTORY4)

                .state(State.TRAJECTORY4)
                .onEnter(()->drive.followTrajectoryAsync(trajectory4))
                .onEnter(()->robot.MoveDumpServo(Robot.DumpServoState.IDLE))
                .transition(()->!drive.isBusy(),State.TRAJECTORY5)

                .state(State.TRAJECTORY5)
                .onEnter(()->drive.followTrajectoryAsync(trajectory5))
                .transition(()->!drive.isBusy(),State.STOP)

                .state(State.STOP)
                .onEnter(()->robot.lift.STARTlIFT(Lift.LiftState.Down))
                .onEnter(()->robot.intake.StopIntaking())

                .build();


        waitForStart();

        if (isStopRequested()) return;

        machine.start();// starts the state machine so we are in the first state

        while (opModeIsActive() && !isStopRequested()) {

            //Update the stateMachine
            machine.update();
            //update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            //Sending pose to poseStroage
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


}