package org.firstinspires.ftc.teamcode.Bistrita.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class BackSpeed{
    private DcMotorEx shaft;
    private DcMotorEx rb;
    private DcMotorEx lb;

    public BackSpeed(HardwareMap hm)
    {
        shaft = hm.get(DcMotorEx.class,"shaft");
        rb = hm.get(DcMotorEx.class,"rb");
        lb = hm.get(DcMotorEx.class,"lb");

        shaft.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        shaft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shaft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType motorConfigurationType = shaft.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shaft.setMotorType(motorConfigurationType);

        MotorConfigurationType motorConfigurationType1 = lb.getMotorType().clone();
        motorConfigurationType1.setAchieveableMaxRPMFraction(1.0);
        lb.setMotorType(motorConfigurationType1);

        MotorConfigurationType motorConfigurationType2 = rb.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        rb.setMotorType(motorConfigurationType2);

        shaft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shaft.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
    public void setPwr(double pwr)
    {
        shaft.setPower(pwr);
        lb.setPower(pwr);
        rb.setPower(pwr);
    }
    public double getSpid()
    {
        return shaft.getVelocity()/(103.8*2*Math.PI*0.065);
    }
}
