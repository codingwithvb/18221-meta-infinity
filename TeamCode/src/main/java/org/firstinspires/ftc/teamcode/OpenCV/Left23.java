/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name="Left23", group="Linear Opmode", preselectTeleOp = "Gamepad2.0")
public class Left23 extends LinearOpMode
{
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    DcMotor rightSlide;
    DcMotor leftSlide;

    Servo clawServo;
    Servo armServo;

    TouchSensor touchSensor;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of Sleeve
    int left = 17;
    int middle = 18;
    int right = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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

        armServo = hardwareMap.servo.get("ArmServo");
        armServo.setPosition(1.0);
        clawServo = hardwareMap.servo.get("ClawServo");
        clawServo.setPosition(1.0);

        touchSensor = hardwareMap.touchSensor.get("Touch");

        Pose2d startPose = new Pose2d(0,0,0);
        robot.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        TrajectorySequence trajLeft = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-30,0))
                .lineToSplineHeading(new Pose2d(-68,0, Math.toRadians(-90)))
                .addTemporalMarker(() -> myGoToHeightPOS(3650,1))
                .addTemporalMarker(() -> armServo.setPosition(0.3))
                .back(2)
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(5)
                .addTemporalMarker(() -> myGoToHeightPOS(-3150, 1))
                .strafeLeft(16)
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-54, -31))
                .addTemporalMarker(() -> clawServo.setPosition(1.0))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> myGoToHeightPOS(2600,1))
                .forward(7)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-49, 10))
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(4)
                .strafeLeft(38)
                .build();
        TrajectorySequence trajMiddle = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-30,0))
                .lineToSplineHeading(new Pose2d(-68,0, Math.toRadians(-90)))
                .addTemporalMarker(() -> myGoToHeightPOS(3650,1))
                .addTemporalMarker(() -> armServo.setPosition(0.3))
                .back(2)
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(5)
                .addTemporalMarker(() -> myGoToHeightPOS(-3150, 1))
                .strafeLeft(16)
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-54, -31))
                .addTemporalMarker(() -> clawServo.setPosition(1.0))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> myGoToHeightPOS(2600,1))
                .forward(7)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-49, 10))
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(4)
                .strafeLeft(15)
                .build();
        TrajectorySequence trajRight = robot.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-30,0))
                .lineToSplineHeading(new Pose2d(-68,0, Math.toRadians(-90)))
                .addTemporalMarker(() -> myGoToHeightPOS(3650,1))
                .addTemporalMarker(() -> armServo.setPosition(0.3))
                .back(2)
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(5)
                .addTemporalMarker(() -> myGoToHeightPOS(-3150, 1))
                .strafeLeft(16)
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-54, -31))
                .addTemporalMarker(() -> clawServo.setPosition(1.0))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> myGoToHeightPOS(2600,1))
                .forward(7)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-49, 10))
                .addTemporalMarker(() -> clawServo.setPosition(0.5))
                .forward(4)
                .strafeRight(14)
                .build();
                
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        
        while (!isStarted() && !isStopRequested())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest.id == left){
            //left code
            telemetry.addLine("Left");
            telemetry.update();
            robot.followTrajectorySequence(trajLeft);
        }else if(tagOfInterest == null || tagOfInterest.id == middle){
            //middle code
            telemetry.addLine("Middle");
            telemetry.update();
            robot.followTrajectorySequence(trajMiddle);
        }else if(tagOfInterest.id == right){
            //right code
            telemetry.addLine("Right");
            telemetry.update();
            robot.followTrajectorySequence(trajRight);
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        telemetry.addData("leftSlidePOS", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlidePOS", rightSlide.getCurrentPosition());
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
        //base encoder code
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition((slidePOS));
        rightSlide.setTargetPosition((slidePOS));
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
        if(leftSlide.getCurrentPosition()>=3800 || leftSlide.getCurrentPosition()<=-3800 || rightSlide.getCurrentPosition()>=3800 || rightSlide.getCurrentPosition()<=-3800) {
            leftSlide.setPower(0.0);
            rightSlide.setPower(0.0);
            telemetry.addData("Maximum Reached", slidePOS);
            telemetry.update();
        }
    }
}
