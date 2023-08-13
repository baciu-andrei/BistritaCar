package org.firstinspires.ftc.teamcode.Bistrita.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Bistrita.modules.BackSpeed;

@TeleOp
@Config
public class car_main extends LinearOpMode {
    public static double p=0.002,i=0.07,d=0.000085;
    public static int angle = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DcMotorEx steering = hardwareMap.get(DcMotorEx.class, "steering");
        BackSpeed motors = new BackSpeed(hardwareMap);


        // steering.setDirection(DcMotorSimple.Direction.REVERSE);
        steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steering.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController pid = new PIDController(p,i,d);

        waitForStart();
        while(opModeIsActive())
        {
            motors.setPwr(gamepad1.right_trigger-gamepad1.left_trigger);

            pid.setPID(p,i,d);

            int target = (int)(gamepad1.left_stick_x)*angle;
            if(gamepad1.right_stick_x!=0) target = (int)(gamepad1.right_stick_x)*100;


            double power = pid.calculate(steering.getCurrentPosition(),target);
            steering.setPower(power);

            telemetry.addData("pos",steering.getCurrentPosition());
            telemetry.addData("targetpos",target);
            telemetry.addData("angle", angle);
            telemetry.addData("spid", motors.getSpid() );
            telemetry.update();
        }
    }
}