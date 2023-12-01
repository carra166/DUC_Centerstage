package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
10430 TELEOP
Motor gamepad controls
*/
@TeleOp
public class TeleopMK2 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoDoor;
    Servo servoWrist;
    CRServo servoIntake;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    boolean canMove = true;

    InterpLUT armAngles;
    double kCos = 0.25;
    double downPower = 0;

    @Override
    public void runOpMode() {

        //initialized motors
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");

        //init imu and variables
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        armAngles = new InterpLUT();
        armAngles.add(-100, -41); //safety 1
        armAngles.add(0, -40); //init position
        armAngles.add(100, 0); //straight out (forward)
        armAngles.add(400, 90); //straight up
        armAngles.add(666, 180); //straight back
        armAngles.add(1000, 181); //safety 2
        armAngles.createLUT();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoWrist.setPosition(0);
        servoDoor.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (this.gamepad1.right_trigger > 0) {
                //slow
                division = 3;
            } else if (this.gamepad1.left_trigger > 0) {
                //fast
                division = 1;
            } else {
                //normal
                division = 2;
            }

            if(gamepad2.left_bumper) {
                // move to 0 degrees.
                servoWrist.setPosition(0);
            } else if (gamepad2.right_bumper) {
                // move to 180 degrees.
                servoWrist.setPosition(1);
            }

            if (gamepad2.a) {
                servoDoor.setPosition(0);
                sleep(750);
                servoDoor.setPosition(0.5);
            }

            if (gamepad2.dpad_up) {
                servoDoor.setPosition(0.5);
            } else if (gamepad2.dpad_down) {
                servoDoor.setPosition(0);
            }

            if (gamepad2.x) {
                servoIntake.setPower(1);
            } else {
                servoIntake.setPower(0);
            }

            if (gamepad2.y) {
                downPower = -0.3;
            } else {
                downPower = 0;
            }

            if(gamepad1.b && canMove) {
                if (tagProcessor.getDetections().size() > 0) {
                    canMove = false;
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    telemetry.addData("AprilTags", "FOUND");
                    telemetry.addData("Range", tag.ftcPose.range);
                    sleep(5000);
                    canMove = true;
                } else {
                    telemetry.addData("AprilTags", "NONE FOUND");
                }
            }

            if (gamepad1.x) {
                while (gamepad1.x) {}

            }

            if (tagProcessor.getDetections().size() > 0 && tagProcessor.getDetections().get(0).metadata != null) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("AprilTags", "FOUND");
                telemetry.addData("Range", tag.ftcPose.range);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                //bearing to the right is positive, left is negative

            } else {
                telemetry.addData("AprilTags", "NONE FOUND");
            }

            tgtPowerForward = (-this.gamepad1.left_stick_y / division);
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division);
            tgtPowerTurn = (this.gamepad1.right_stick_x / division);
            tgtPowerArm = ((this.gamepad2.left_stick_y / -5) + calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos) + downPower );

            //sets motor powereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew44444444444444    q1`1`
            //this comment corrupted but its just too funny to delete
            if (canMove) {
                setPowerAll(
                        -tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        -tgtPowerForward - tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward - tgtPowerStrafe - tgtPowerTurn
                );
            }
            armMotor.setPower(tgtPowerArm);
            armMotor2.setPower(-tgtPowerArm);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("current wrist amount", servoWrist.getPosition());
            telemetry.addData("right stick y", this.gamepad1.right_stick_y);

            telemetry.addData("current arm position", armMotor.getCurrentPosition());
            telemetry.addData("arm angle", armAngles.get(armMotor.getCurrentPosition()));
            telemetry.addData("gravity compensation", calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos));
            telemetry.addData("current tgt arm", tgtPowerArm);
            telemetry.addData("current arm motor power", armMotor.getPower());
            telemetry.update();
        }
    }


    private void setPowerAll(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void initializeMotors() {
        armMotor = hardwareMap.get(DcMotor.class, "ARM1");
        armMotor2 = hardwareMap.get(DcMotor.class, "ARM2");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoDoor = hardwareMap.get(Servo.class, "door");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoIntake = hardwareMap.get(CRServo.class,"intake");

    }

    private double calculateArmPower(double armAngle, double kCos) {
        return kCos * Math.cos(Math.toRadians(armAngle));
    }

}