package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ICarModule;
import org.firstinspires.ftc.teamcode.utils.DumbEncoder;

public abstract class PowerTrain implements ICarModule {
    public DumbEncoder powerTrainEncoder;

    public abstract void setSpeed(double input);
    public abstract void setRawSpeed(double input);

    public abstract void setMotorPower();

    public abstract double getProfileSpeed();
}
