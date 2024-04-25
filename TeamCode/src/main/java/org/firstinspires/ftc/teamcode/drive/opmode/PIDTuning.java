package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDTuning extends LinearOpMode {
    private PIDController controller;

    public static double p =0,i=0,d=0;

    public static double target = 0;

    private final double ticks_in_degree = 1800/360;

    private DcMotorEx motor;

    @Override
    public void runOpMode(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "MotorName");

        waitForStart();

        while (opModeIsActive()){
            controller.setPID(p,i,d);
            int Armpose = motor.getCurrentPosition();
            double pid = controller.calculate(Armpose, target);

            double power = pid ;

            motor.setPower(power);

            telemetry.addData("pos ",Armpose);
            telemetry.addData("target ",target);
            telemetry.update();

        }
    }
}
