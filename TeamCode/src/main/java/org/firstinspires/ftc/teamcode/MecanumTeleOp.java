package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

/**
 * First TeleOp
 */
@TeleOp(name = "Basic OpMode", group="TeleOp Iterative Opmode")
//@Disabled
public class MecanumTeleOp extends OpMode {


    private IMU imu;
    private double initAngle;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");
        initAngle = imu.getHeading();
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {

    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y * -1;
        double turn = gamepad1.left_stick_x;

        telemetry.addData("Strafe", strafe);
        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", turn);
        telemetry.addData("IMU", imu.getHeading());
        telemetry.update();
    }
}
