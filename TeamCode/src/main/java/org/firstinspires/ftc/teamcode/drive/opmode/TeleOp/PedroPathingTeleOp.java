package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.PoseStoragePedroPathing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
@TeleOp(name = "Pedro Pathing TeleOp ")
public class PedroPathingTeleOp extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Vector driveVector;
    private Vector headingVector;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);
        follower.setPose(PoseStoragePedroPathing.currentpose);
        leftFront = hardwareMap.get(DcMotorEx.class, "motorFL");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorBL");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorBR");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorFR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveVector = new Vector();
        headingVector = new Vector();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(follower.getCentripetalForceCorrection(), headingVector, driveVector);
        follower.update();


    }

}
