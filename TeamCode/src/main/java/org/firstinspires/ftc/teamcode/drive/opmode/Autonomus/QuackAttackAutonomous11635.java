package org.firstinspires.ftc.teamcode.drive.opmode.Autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PID;
import org.firstinspires.ftc.teamcode.drive.opmode.PidF;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

@Autonomous(name = "Auto1")
public class QuackAttackAutonomous11635 extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo trajectory and go to pixel stack
        TRAJECTORY_2,   // Then start intake and go forward
        WAIT_1,         // Then we want to wait for the intake to intake and then stop it
        TRAJECTORY_3,   // Then, we follow another lineToLinearHeading trajectory and go to the backdrop
        TRAJECTORY4,    // Then we're gonna strafeRight
        TRAJECTORY5,    // Finally, we're gonna go forward again again and park in the parking zone
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(-40, -64, Math.toRadians(0));

    PID UP_LiftpidController = new PID();
    PID Down_LiftpidController = new PID();
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift and intake
        UP_LiftpidController.BeforeInitPid(6000,1,1,1,"Lift",1800,0);//all the vairables for the pidcontroller
        UP_LiftpidController.InitPid();//Init pidcontroller

        Down_LiftpidController.BeforeInitPid(0,1,1,1,"Lift",1800,0);//all the vairables for the pidcontroller
        Down_LiftpidController.InitPid();//Init pidcontroller

        DcMotor Intake = hardwareMap.get(DcMotor.class,"intake");
        DcMotor Intake2 = hardwareMap.get(DcMotor.class,"intakeWheel");

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-55, -35), Math.toRadians(90))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(5)
                .build();

        // Define a 1 second wait time
        double waitTime1 = 1;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // For turning :Define the angle to turn at
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


        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        //Going to state "TRAJECTORY_2" and starting intake and following Trajectory2
                        currentState = State.TRAJECTORY_2;
                        Intake.setPower(1);
                        Intake2.setPower(1);
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following  trajectory2
                    // Move on to the next state, WAIT_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has finished
                    // If it did stop the intake and  move onto the next state, TRAJECTORY_3
                    if (waitTimer1.seconds() >= waitTime1) {
                        Intake.setPower(0);
                        Intake2.setPower(0);
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following trajectory3
                    // If not, move onto the next state, TRAJECTORY4
                    UP_LiftpidController.LoopPid();//lifting the lift to high level while it still follows trajectory3
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY4;
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case TRAJECTORY4:
                    // Check if the drive class is busy following trajectory4
                    // If not, move onto the next state, TRAJECTORY5
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY5;
                        drive.followTrajectoryAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY5:
                    // Check if the drive class is busy following trajectory5
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    Down_LiftpidController.LoopPid();//Keep the lift at it starting position while finished auto
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


}