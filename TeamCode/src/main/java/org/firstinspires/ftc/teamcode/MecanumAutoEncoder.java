package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name="Mecanum Encoder", group="Autonomous Linear Opmode")
//@Disabled
public class MecanumAutoEncoder extends LinearOpMode {

    private IMU imu;

    public void initialize(){

        // IMU (Inertial Measurement Unit)
        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();
        telemetry.addData("Initialized", true);
        telemetry.addData("IMU", imu.getAngle());

    }
}
