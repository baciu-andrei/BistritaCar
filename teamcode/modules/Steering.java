package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ICarModule;
import org.firstinspires.ftc.teamcode.utils.DumbEncoder;

import fi.iki.elonen.NanoHTTPD;

public class Steering implements ICarModule {

    private DcMotorEx steeringMotor;
    public DumbEncoder steeringEncoder;

    public static String STEERING_MOTOR_NAME = "steering";
    public static String STEERING_ENCODER_NAME = "steeringEncoder";
    public static boolean reversed = true, reversedEnc = false;

    public Steering(HardwareMap hm, boolean resetEncoder){
        steeringMotor = hm.get(DcMotorEx.class, STEERING_MOTOR_NAME);
        steeringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed) steeringMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        steeringEncoder = new DumbEncoder(hm, STEERING_ENCODER_NAME, reversedEnc);
        if(resetEncoder) steeringEncoder.reset();
    }

    public static double TICK_PER_REV = 8192;
    public static double steeringAngleLimit = 30;

    public double steeringAngle = 0;
    public double steeringTargetPosition = 0;

    public void setSteeringAngle(double steeringInput){
        this.steeringAngle = steeringInput * steeringAngleLimit;
        this.steeringTargetPosition = steeringInput * TICK_PER_REV * (steeringAngleLimit/360.0);
    }

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0,0,0,0);
    private final PIDController pid = new PIDController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);

    public double steeringPower = 0;

    public double steeringCurrentPosition = 0;

    public void setSteeringPower(){
        pid.setPID(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
        steeringCurrentPosition = steeringEncoder.getCurrentPosition();
        this.steeringPower = pidfCoefficients.f*Math.signum(steeringTargetPosition - steeringCurrentPosition) + pid.calculate(steeringCurrentPosition, steeringTargetPosition);
        steeringMotor.setPower(steeringPower);
    }

    @Override
    public void loop() {
        setSteeringPower();
    }

    @Override
    public void emergencyStop() {
        steeringMotor.setPower(0);
    }
}
