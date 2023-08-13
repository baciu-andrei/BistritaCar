package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.CarModules;
import org.firstinspires.ftc.teamcode.ICarModule;

public class GamepadControl implements ICarModule {

    public static boolean ENABLE_MODULE = true;

    public Gamepad gamepad1, gamepad2;
    private final CarModules car;

    public GamepadControl(Gamepad gamepad1, Gamepad gamepad2, CarModules car){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.car = car;
    }

    public static double correctionThreshold = 0.05;

    private double targetHeading = 0;
    public double pidCorrection = 0;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    private PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

    private void steering(){

        if(Math.abs(gamepad1.left_stick_x) > correctionThreshold) {
            car.steering.setSteeringAngle(gamepad1.left_stick_x);
            targetHeading = car.imu.getHeading();
        }
        else
        {
            pid.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
            pidCorrection = Math.signum(car.pt.getProfileSpeed())*pid.calculate(car.imu.getHeading(), targetHeading);
            pidCorrection = Math.min(1, pidCorrection);
            pidCorrection = Math.max(-1, pidCorrection);
            car.steering.setSteeringAngle(pidCorrection);
        }


    }

    private void powerTrainControl(){
        car.pt.setSpeed(gamepad1.right_trigger - gamepad1.left_trigger);
//        car.pt.setRawSpeed(-gamepad1.right_stick_y);
    }

    @Override
    public void loop() {
        steering();
        powerTrainControl();
    }

    @Override
    public void emergencyStop() {

    }
}
