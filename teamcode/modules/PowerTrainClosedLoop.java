package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ICarModule;
import org.firstinspires.ftc.teamcode.utils.DumbEncoder;

public class PowerTrainClosedLoop extends PowerTrain implements ICarModule {
    public static boolean ENABLE_MODULE = true;

    private DcMotorEx motor1, motor2, motor3;

    public static String POWER_TRAIN_MOTOR_NAME_PREFIX = "powerTrain";
    public static String POWER_TRAIN_ENCODER_NAME = "powerTrainEncoder";
    public static boolean reversed1 = false, reversed2 = false, reversed3 = false, reversedEnc = false;

    public PowerTrainClosedLoop(HardwareMap hm, boolean resetEncoder){
        motor1 = hm.get(DcMotorEx.class, POWER_TRAIN_MOTOR_NAME_PREFIX + "1");
        motor2 = hm.get(DcMotorEx.class, POWER_TRAIN_MOTOR_NAME_PREFIX + "2");
        motor3 = hm.get(DcMotorEx.class, POWER_TRAIN_MOTOR_NAME_PREFIX + "3");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed1) motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        if(reversed2) motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        if(reversed3) motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        super.powerTrainEncoder = new DumbEncoder(hm, POWER_TRAIN_ENCODER_NAME, reversedEnc);
        if(resetEncoder) super.powerTrainEncoder.reset();
    }

    public double motorPower = 0;

    private double lastLoop = -1;

    public static double accelerationSlope = 0.00005;
    public static double decelerationSlope = 0.0005;

    @Override
    public void setSpeed(double input) {
        if(lastLoop == -1) lastLoop = System.nanoTime();
        double elapsedTime = (System.nanoTime() - lastLoop)/1e6;
        lastLoop = System.nanoTime();
        if(motorPower < input){
            if(motorPower < 0) motorPower += Math.min(decelerationSlope * elapsedTime, input-motorPower);
            else motorPower += Math.min(accelerationSlope * elapsedTime, input-motorPower);
        }
        else if(motorPower > input){
            if(motorPower < 0) motorPower -= Math.min(accelerationSlope * elapsedTime, motorPower - input);
            else motorPower -= Math.min(decelerationSlope * elapsedTime, motorPower - input);
        }
        else motorPower = 0;
        motorPower = Math.max(-1, motorPower);
        motorPower = Math.min(1, motorPower);
    }

    @Override
    public void setRawSpeed(double input) {
        motorPower = 0;
    }

    //all units meters and seconds where not specified
    public static double maxSpeed = 1;
    public static double wheelRadius = 0.036;
    public static double maxSpeedTicks = 0;

    public static double TICKS_PER_REV = 28.0*3.7;

    PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

    public static double rawMotorPower = 0;

    ElapsedTime pidTimer;

    void updateMotorPower(){
        maxSpeedTicks = maxSpeed/(2*Math.PI*wheelRadius) * TICKS_PER_REV;

        double current = powerTrainEncoder.getVelocity();
        double target = motorPower * maxSpeedTicks;

        pid.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        rawMotorPower += pid.calculate(current, target) * pidTimer.seconds();
        pidTimer.reset();
    }

    @Override
    public void setMotorPower(){
        motor1.setPower(rawMotorPower);
        motor2.setPower(rawMotorPower);
        motor3.setPower(rawMotorPower);
    }

    @Override
    public double getProfileSpeed() {
        return motorPower;
    }

    @Override
    public void loop() {
        updateMotorPower();
        setMotorPower();
    }

    @Override
    public void emergencyStop() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
}
