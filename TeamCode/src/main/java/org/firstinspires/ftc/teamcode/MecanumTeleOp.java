package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    //private Servo servo0;
    private DcMotor fr, fl, br, bl;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        fr = hardwareMap.get(DcMotor.class, "fr_motor");
        fl = hardwareMap.get(DcMotor.class, "fl_motor");
        br = hardwareMap.get(DcMotor.class, "br_motor");
        bl = hardwareMap.get(DcMotor.class, "bl_motor");

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //servo0 = hardwareMap.get(Servo.class, "servo0");

        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");
        initAngle = imu.getAngle();
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

        double drive = gamepad1.right_stick_y * -1;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        //servo0.setPosition(Math.abs(drive));

        fr.setPower(drive - strafe - turn);
        fl.setPower(drive + strafe + turn);
        br.setPower(drive + strafe - turn);
        bl.setPower(drive - strafe + turn);

        //telemetry.addData("Servo0", servo0.getPosition());
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", turn);
        telemetry.addData("IMU", imu.getAngle());
        telemetry.update();
    }
}
