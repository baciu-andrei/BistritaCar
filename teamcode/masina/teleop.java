package org.firstinspires.ftc.teamcode.masina;



import static org.firstinspires.ftc.teamcode.masina.teleop.tele;
import static org.firstinspires.ftc.teamcode.masina.teleop.angle;
import static org.firstinspires.ftc.teamcode.masina.teleop.d;
import static org.firstinspires.ftc.teamcode.masina.teleop.ds;
import static org.firstinspires.ftc.teamcode.masina.teleop.i;
import static org.firstinspires.ftc.teamcode.masina.teleop.is;
import static org.firstinspires.ftc.teamcode.masina.teleop.p;
import static org.firstinspires.ftc.teamcode.masina.teleop.ps;
import static org.firstinspires.ftc.teamcode.masina.teleop.targetHeading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ImuModule;

import java.util.List;

@TeleOp(name="masinacaaaar👉👌")
@Config
public class teleop extends LinearOpMode {
    public static double p=0.002,i=0.03,d=0.000085;
    public static double ps=0,is=0,ds=0;
    public static int angle = 750;
    public static double correctionThreshold = 0.05;
    public static double targetHeading=0;
    public static Telemetry tele;
    List<LynxModule> hubs;
    FtcDashboard dash;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.disable();
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        Speed speed = new Speed(hardwareMap);
        Steering steering = new Steering(hardwareMap);
        ImuModule imu = new ImuModule(hardwareMap);
        tele = telemetry;
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        imu.startIMUThread(this);
        targetHeading = imu.getHeading();
        SteeringCorrection correction = new SteeringCorrection(imu,steering);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        while(opModeIsActive()){
            speed.power((gamepad1.right_trigger*gamepad1.right_trigger)-(gamepad1.left_trigger*gamepad1.left_trigger));
//            if(Math.abs(gamepad1.left_stick_x)>correctionThreshold) {
                steering.steer(gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x);
                targetHeading = imu.getHeading();
//            }
//            else{
//                correction.correct();
//            }



            telemetry.addData("speed: m/s",speed.getSpeed());
            telemetry.update();
        }

    }
}

class SteeringCorrection{
    ImuModule imu;
    Steering steering;
    PIDController pid;
    public SteeringCorrection(ImuModule imu,Steering steering){
        this.imu = imu;
        this.steering = steering;
        pid = new PIDController(ps, is, ds);
    }

    public void correct(){

        double currentHeading = imu.getHeading();
        pid.setPID(ps,is,ds);
        double error1 = targetHeading - currentHeading;
        double error2 = Math.signum(error1)*-1*(Math.PI*2 - Math.abs(error1));

        if(Math.abs(error1) > Math.abs(error2)) error1 = error2;

        double pidCorrection = pid.calculate(0,error1);
        pidCorrection = Math.min(1, pidCorrection);
        pidCorrection = Math.max(-1, pidCorrection);
        steering.steer(pidCorrection);

        tele.addData("currentHeading",currentHeading);
        tele.addData("targetHeading",targetHeading);
        tele.addData("error1",error1);
        tele.addData("error2",error2);
    }

}

class Steering{
    DcMotorEx steering;
    PIDController pid;

    public Steering(HardwareMap hm){
        steering = hm.get(DcMotorEx.class, "steer");
        steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steering.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p,i,d);
    }

    public void steer(double pos){

        int target = (int)(pos * angle);
        pid.setPID(p,i,d);
        double power = pid.calculate(steering.getCurrentPosition(),target);
        steering.setPower(power);
        tele.addData("currentSteering",steering.getCurrentPosition());
        tele.addData("targetSteering",target);
    }

}

class Speed{
    DcMotorEx m1;
    DcMotorEx m2;
    DcMotorEx m3;

    public Speed(HardwareMap hm){
        m1 = hm.get(DcMotorEx.class, "m1");
        m2 = hm.get(DcMotorEx.class,"m2");
        m3 = hm.get(DcMotorEx.class,"m3");
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType motorConfigurationType = m1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m1.setMotorType(motorConfigurationType);
        motorConfigurationType = m2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m2.setMotorType(motorConfigurationType);
        motorConfigurationType = m3.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m3.setMotorType(motorConfigurationType);

    }

    public void power(double p){
        m1.setPower(p);
        m2.setPower(p);
        m3.setPower(p);
    }

    public double getSpeed(){
        return m2.getVelocity()/103.6*0.056*Math.PI;
    }
}
