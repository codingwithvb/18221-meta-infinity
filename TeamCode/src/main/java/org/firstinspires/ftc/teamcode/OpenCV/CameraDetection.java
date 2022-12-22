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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.concurrent.ForkJoinPool;

@Autonomous
public class CameraDetection extends LinearOpMode
{
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
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
        FrontLeft  = hardwareMap.dcMotor.get("LeftFront");
        FrontRight = hardwareMap.dcMotor.get("RightFront");
        BackRight = hardwareMap.dcMotor.get("RightBack");
        BackLeft = hardwareMap.dcMotor.get("LeftBack");

        leftIntakeMotor = hardwareMap.dcMotor.get("LeftIntake");
        rightIntakeMotor = hardwareMap.dcMotor.get("RightIntake");

        leftSlide = hardwareMap.dcMotor.get("LeftSlide");
        rightSlide = hardwareMap.dcMotor.get("RightSlide");

        armServo = hardwareMap.servo.get("ArmServo");
        armServo.setPosition(1.0);
        clawServo = hardwareMap.servo.get("ClawServo");
        clawServo.setPosition(1.0);

        touchSensor = hardwareMap.touchSensor.get("Touch");




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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            /*if(tagOfInterest.id == left){
                //left code
                telemetry.addLine("Left");
                telemetry.update();
            }else if(tagOfInterest == null ||  tagOfInterest.id == middle){
                //middle code
                telemetry.addLine("Middle");
                telemetry.update();
            }else if(tagOfInterest.id == right){
                //right code
                telemetry.addLine("Right");
                telemetry.update();
            }*/


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
            EverythingBeforePark();
        }else if(tagOfInterest == null || tagOfInterest.id == middle){
            //middle code
            telemetry.addLine("Middle");
            telemetry.update();
        }else if(tagOfInterest.id == right){
            //right code
            telemetry.addLine("Right");
            telemetry.update();
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
    private void myGoToPOS(int LF, int LB, int RB, int RF, double motorPower) {
     FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FrontLeft.setTargetPosition(LF);
        FrontRight.setTargetPosition(RF);
        BackRight.setTargetPosition(RB);
        BackLeft.setTargetPosition(LB);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        FrontLeft.setPower(motorPower);
        FrontRight.setPower(motorPower);
        BackRight.setPower(motorPower);
        BackLeft.setPower(motorPower);
      while (FrontLeft.isBusy() || FrontRight.isBusy() || BackRight.isBusy() || BackLeft.isBusy()) {
          telemetry.addData("LeftFront", FrontLeft.getCurrentPosition());
            telemetry.addData("RightFront", FrontRight.getCurrentPosition());
            telemetry.addData("RightBack", BackRight.getCurrentPosition());
            telemetry.addData("LeftBack", BackLeft.getCurrentPosition());
           telemetry.addData("LeftFront", LF);
           telemetry.addData("RightFront", RF);
           telemetry.addData("RightBack", RB);
           telemetry.addData("LeftBack", LB);
           telemetry.update();
      }
    }
    private void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(slidePOS);
        rightSlide.setTargetPosition(slidePOS);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(motorPower);
        rightSlide.setPower(motorPower);
        while(leftSlide.isBusy()) {
            telemetry.addData("Slide", leftSlide.getCurrentPosition());
            telemetry.update();
        }
    }
    public void drop() {
        clawServo.setPosition(0.5);
        sleep(500);
        clawServo.setPosition(1.0);
}
    public void EverythingBeforePark(){
        //myGoToPOS(-2000,-2000,-2000,-2000,.2);
        //myGoToPOS(-1000, -1000, 0, 0, .2);
        myGoToHeightPOS(3450, .4);
        armServo.setPosition(0.2);
        sleep(800);
        drop();
        myGoToHeightPOS(-3450, .5);
    }
}
