package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import net.waring.java4ftc.ops.IMU;
import net.waring.java4ftc.ops.SyncTask;
import net.waring.java4ftc.utilities.Utils;


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


        drive(10000, 0.0, () -> servo.setPosition(Math.random()));

        turn(-180, 0.5);

        drive(9000, 0.0, null);

        turn(0, 0.5);

        drive(5000, 0.0, null);

        turn(-180, 0.5);

        drive(4000, 0.0, () -> servo.setPosition(Math.random()));

        turn(0, 0.5);

        drive(2000, 0.0, () -> servo.setPosition(Math.random()));

        sleep(1000);

        drive(1000, 0.0, () -> servo.setPosition(Math.random()));

    }

    /**
     * @param targetAngle
     * @param MOE
     */
    private void turn(double targetAngle, double MOE) {

        System.out.println("Turning to " + targetAngle + " degrees");

        telemetry.addData("Target Angle", targetAngle);
        telemetry.update();

        double currentAngle         = imu.getHeading();
        double deltaAngle           = Math.abs(targetAngle - currentAngle);
        double power                = (targetAngle > currentAngle) ? 1 : -1;
        double minPower             = 0.1 * power;
        double maxPower             = 0.6 * power;

        // Retrieve angle and MOE
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        while (lowerBound >= currentAngle || currentAngle >= upperBound){

            // Power Ramping based off a sin method
            double currentDeltaAngle = targetAngle - currentAngle;
            double rampedPower = maxPower * Math.sin(Math.PI * Math.abs(currentDeltaAngle / deltaAngle));
            power = rampedPower + minPower;


            // Handle clockwise (+) and counterclockwise (-) motion
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
            //System.out.println("Power: " + power);

            currentAngle = imu.getHeading();
        }
        // Stop power
        setAllPower(0);
    }





    /**
     * Drive straight with Proportional PID Controller
     * @param ticks
     */
    private void drive(int ticks, Double targetAngle, SyncTask task) {

        System.out.println("Driving " + ticks + " ticks");

        // Initialize power settings
        double leftPower        = 0;
        double rightPower       = 0;
        double learning_rate    = 0.000001;

        // thresholds are dependent on acceleration
        double acceleration = 0.3;
        double power = 0;

        /*
        Smaller distances should be slower and have more transition. Therefore ticks is inversely proportional to threshold value.
        Since the ramping takes at most 2000 ticks, we specify that if we have 2000 ticks, we should be ramping up half the distance,
        and ramping down the other half.
        Or if we specify 10000 ticks, then we should ramp up for 10% of the distance, and ramp down for 10% of the distance, leaving 80% for full power
         */
        double threshold = 0.1 / acceleration; // Threshold is the percentage of the distance we should be ramping power
        double accelerateThreshold = threshold * ticks;
        double decelerateThreshold = (1 - threshold) * ticks;
        double normFactor = 1 / Math.sqrt(acceleration * accelerateThreshold); // retrieve max value, normalize it to 1
        double maxPower = -1;

        resetMotors();
        setAllPower(power);

        if (targetAngle == null) targetAngle = imu.getHeading();
        double position = getPosition();
        while(position < ticks){

            if (task != null) task.execute();

            if (power > maxPower) maxPower = power;

            // Modeling a piece wise of power as a function of distance
            if (position <= accelerateThreshold)                                                   power = normFactor * Math.sqrt(acceleration * position);
            else if (position > accelerateThreshold && position < decelerateThreshold)             power = maxPower;
            else if (position >= decelerateThreshold)                                              power = normFactor * Math.sqrt(acceleration * (ticks - position));

            if (power > 1) throw new Error("Power is over 1");

            setAllPower(power);
            position =  getPosition() + 0.0001;



            // Logging
            telemetry.addData("Position", position);
            telemetry.addData("Power", power);
            telemetry.addData("Traveled", position / ticks * 100);
            telemetry.update();






            // Direction Tuning [ PID Controller ]
/*            double currentHeading = imu.getHeading();
            double error = targetAngle - currentHeading;
            double correction = error * learning_rate;
            leftPower -= correction;
            rightPower += correction;

            fl.setPower(leftPower);
            bl.setPower(leftPower);
            fr.setPower(rightPower);
            br.setPower(rightPower);*/
        }
        setAllPower(0);
    }

    double getThreshold(double ticks){
        //return (0.05 * ticks + 500) / ticks;
        return 0.0001 * (-Math.sqrt(10 * ticks) + 500);
    }

    double getPosition(){
        return (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
    }

    public double powerFromTicks(double x, double accelerationFactor){
        return Math.sqrt(x * accelerationFactor);
    }


    /**
     * @param power
     */
    private void setAllPower(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * Resets Motors
     */
    private void resetMotors(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Maps value to a given range
     * @param value
     * @param min
     * @param max
     * @return
     */
    private double map(double value, double min, double max){
        return (value - min) / (max - min);
    }


}
