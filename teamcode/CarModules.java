package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.PowerTrain;
import org.firstinspires.ftc.teamcode.modules.PowerTrainClosedLoop;
import org.firstinspires.ftc.teamcode.modules.PowerTrainOpenLoop;
import org.firstinspires.ftc.teamcode.modules.Steering;
import org.firstinspires.ftc.teamcode.utils.ImuModule;

public class CarModules implements ICarModule{

    public PowerTrain pt;
    public Steering steering;
    public ImuModule imu;

    public CarModules(HardwareMap hm, boolean closedLoopPowerTrain, boolean resetEncoders){
        if(closedLoopPowerTrain) pt = new PowerTrainClosedLoop(hm, resetEncoders);
        else pt = new PowerTrainOpenLoop(hm, resetEncoders);

        steering = new Steering(hm, resetEncoders);

        imu = new ImuModule(hm);
    }

    @Override
    public void loop() {
        pt.loop();
        steering.loop();
    }

    @Override
    public void emergencyStop() {
        pt.emergencyStop();
        steering.emergencyStop();
    }

    private void ptProfileTelemetry(Telemetry telemetry){
        telemetry.addData("Profile speed", pt.getProfileSpeed());
    }

    private void velocityPidTelemetry(Telemetry telemetry){
        telemetry.addData("Target speed", pt.getProfileSpeed() * PowerTrainClosedLoop.maxSpeedTicks);
        telemetry.addData("Current speed", pt.powerTrainEncoder.getCurrentPosition());
    }

    private void steeringPidTelemetry(Telemetry telemetry){
        telemetry.addData("Steering Current Position", steering.steeringCurrentPosition);
        telemetry.addData("Steering Target Position", steering.steeringTargetPosition);
    }

    public void telemetry(Telemetry telemetry) {
        ptProfileTelemetry(telemetry);
        velocityPidTelemetry(telemetry);
        steeringPidTelemetry(telemetry);
    }
}
