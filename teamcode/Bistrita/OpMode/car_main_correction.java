package org.firstinspires.ftc.teamcode.Bistrita.OpMode;

import static org.firstinspires.ftc.teamcode.masina.teleop.ds;
import static org.firstinspires.ftc.teamcode.masina.teleop.is;
import static org.firstinspires.ftc.teamcode.masina.teleop.ps;
import static org.firstinspires.ftc.teamcode.masina.teleop.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bistrita.modules.BackSpeed;
import org.firstinspires.ftc.teamcode.utils.ImuModule;

@TeleOp
@Config
public class car_main_correction extends LinearOpMode {
    public static double p=0.002,i=0.07,d=0.000085;
    public static double ps=0,is=0,ds=0;
    public static double threshold = 0.05;
    public static int angle = 300;
    public static int angleCorrection = 150;
    public double targetHeading=0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DcMotorEx steering = hardwareMap.get(DcMotorEx.class, "steering");
        BackSpeed motors = new BackSpeed(hardwareMap);
        ImuModule imu = new ImuModule(hardwareMap);


        steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steering.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController pid = new PIDController(p,i,d);
        PIDController pids = new PIDController(ps,is,ds);

        waitForStart();
        imu.startIMUThread(this);
        targetHeading = imu.getHeading();
        while(opModeIsActive())
        {
            motors.setPwr(gamepad1.right_trigger-gamepad1.left_trigger);

            pid.setPID(p,i,d);
            pids.setPID(ps,is,ds);

            int target = (int)(gamepad1.left_stick_x)*angle;
            if(gamepad1.right_stick_x!=0) target = (int)(gamepad1.right_stick_x)*100;

            double currentHeading=0;
            double power = pid.calculate(steering.getCurrentPosition(), target);
            if(!(Math.abs(gamepad1.left_stick_x)>=threshold)) {

                currentHeading = imu.getHeading();
                pids.setPID(ps,is,ds);
                double error1 = targetHeading - currentHeading;
                double error2 = Math.signum(error1)*-1*(Math.PI*2 - Math.abs(error1));

                if(Math.abs(error1) > Math.abs(error2)) error1 = error2;

                double pidCorrection = pids.calculate(0,error1);
                pidCorrection = Math.min(1, pidCorrection);
                pidCorrection = Math.max(-1, pidCorrection);
                telemetry.addData("pula", pidCorrection);
                target = (int)(pidCorrection*angleCorrection);
                power = pid.calculate(steering.getCurrentPosition(),-target);
            }else{
                targetHeading=imu.getHeading();
            }
            steering.setPower(power);

            telemetry.addData("pos",steering.getCurrentPosition());
            telemetry.addData("targetpos",target);
            telemetry.addData("angle", angle);
            telemetry.addData("spid", motors.getSpid() );
            telemetry.addData("heading",currentHeading);
            telemetry.addData("targetHeading",targetHeading);
            telemetry.update();
        }
    }
}