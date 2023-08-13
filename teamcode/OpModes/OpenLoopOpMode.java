package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CarModules;
import org.firstinspires.ftc.teamcode.modules.GamepadControl;
import org.firstinspires.ftc.teamcode.TiedBehaviour;

import java.util.List;


@TeleOp(name="OLOpMode👉👌")
public class OpenLoopOpMode extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    CarModules carModules;
    GamepadControl gamepadControl;
    TiedBehaviour tiedBehaviour;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        carModules = new CarModules(hardwareMap, false, true);
        gamepadControl = new GamepadControl(gamepad1, gamepad2, carModules);
        tiedBehaviour = new TiedBehaviour(carModules, gamepadControl);
    }

    @Override
    public void runOpMode()  {
        initialize();

        waitForStart();

        carModules.imu.startIMUThread(this);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        while(opModeIsActive() && !isStopRequested()) {
            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            gamepadControl.loop();
            carModules.loop();
            tiedBehaviour.loop();

            carModules.telemetry(telemetry);

            telemetry.update();
        }

        carModules.emergencyStop();
    }
}