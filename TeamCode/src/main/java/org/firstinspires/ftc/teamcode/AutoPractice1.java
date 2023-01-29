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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoPractice1", group="Linear Opmode")
//@Disabled
public class AutoPractice1 extends LinearOpMode {
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    DcMotor rightSlide;
    DcMotor leftSlide;

    Servo clawServo;
    Servo armServo;
    Servo armServo2;

    NormalizedColorSensor colorSensor;

    double gc_clawOpen = 0.5;      // Claw Open
    double gc_clawClosed = 0.7;     // Claw Closed

    public void runOpMode() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        FrontLeft  = hardwareMap.dcMotor.get("LeftFront");
        FrontRight = hardwareMap.dcMotor.get("RightFront");
        BackRight = hardwareMap.dcMotor.get("RightBack");
        BackLeft = hardwareMap.dcMotor.get("LeftBack");

        leftSlide = hardwareMap.dcMotor.get("LeftSlide");
        rightSlide = hardwareMap.dcMotor.get("RightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        armServo = hardwareMap.servo.get("ArmServo");
        armServo2 = hardwareMap.servo.get("ArmServo2");
        armServo2.setDirection(Servo.Direction.REVERSE);
        setArmServoPOS(1);
        clawServo = hardwareMap.servo.get("ClawServo");
        clawServo.setPosition(0.7);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }


        Pose2d startPose = new Pose2d(0,0,0);
        robot.setPoseEstimate(startPose);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        waitForStart();

        TrajectorySequence traj = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-40, 0))
                .lineToSplineHeading(new Pose2d(-55, -3, Math.toRadians(45)))
                .addTemporalMarker(() -> myGoToHeightPOS(2500, 0.75))
                .addTemporalMarker(() -> setArmServoPOS(0))
                .waitSeconds(1.25)
                .addTemporalMarker(() -> coneDeposit(600))
                .lineToSplineHeading(new Pose2d(-49, 27.5, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    clawServo.setPosition(gc_clawClosed);
                    sleep(500);
                    myGoToHeightPOS(1500, 0.75);
                })
                .lineToSplineHeading(new Pose2d(-54, -3, Math.toRadians(45)))
                .addTemporalMarker(() -> myGoToHeightPOS(2500, 0.75))
                .addTemporalMarker(() -> setArmServoPOS(0))
                .waitSeconds(1.25)
                .addTemporalMarker(() -> coneDeposit(0))
                .build();

        robot.followTrajectorySequence(traj);
    } // runOpMode()
    public void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        telemetry.addData("leftSlide", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide", rightSlide.getCurrentPosition());
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
        //base encoder code
        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(slidePOS);
        rightSlide.setTargetPosition(slidePOS);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(motorPower);
        rightSlide.setPower(motorPower);
        while(leftSlide.isBusy() || rightSlide.isBusy()){
            //to find slide position and motor position
            telemetry.addData("leftSlidePOS", leftSlide.getCurrentPosition());
            telemetry.addData("rightSlidePOS", rightSlide.getCurrentPosition());
            telemetry.addData("motorPower", motorPower);
            telemetry.update();
        }
    }
    public void setArmServoPOS(double servoPos) {
        armServo.setPosition(servoPos);
        armServo2.setPosition(servoPos);
    }
    public void coneDeposit(int slidePOS){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        clawServo.setPosition(gc_clawOpen);
        sleep(500);
        while(colors.red>0.03 && colors.blue>0.025){
            telemetry.update();
        }
        clawServo.setPosition(gc_clawClosed);
        setArmServoPOS(1.0);
        sleep(1000);
        myGoToHeightPOS(slidePOS, 1);
        setArmServoPOS(1);
        sleep(500);
        clawServo.setPosition(gc_clawOpen);
    }
}
