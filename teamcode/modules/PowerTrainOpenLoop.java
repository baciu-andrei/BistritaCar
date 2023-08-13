package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ICarModule;
import org.firstinspires.ftc.teamcode.utils.DumbEncoder;

public class PowerTrainOpenLoop extends PowerTrain implements ICarModule {
    public static boolean ENABLE_MODULE = true;

    private DcMotorEx motor1, motor2, motor3;

    public static String POWER_TRAIN_MOTOR_NAME_PREFIX = "powerTrain";
    public static String POWER_TRAIN_ENCODER_NAME = "powerTrainEncoder";
    public static boolean reversed1 = false, reversed2 = false, reversed3 = false, reversedEnc = false;

    public PowerTrainOpenLoop(HardwareMap hm, boolean resetEncoder){
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
        motor1.setPower(input);
        motor2.setPower(input);
        motor3.setPower(input);
    }

    @Override
    public void setMotorPower() {
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);
        motor3.setPower(motorPower);
    }

    @Override
    public double getProfileSpeed() {
        return motorPower;
    }

    @Override
    public void loop() {
        setMotorPower();
    }

    @Override
    public void emergencyStop() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
}
