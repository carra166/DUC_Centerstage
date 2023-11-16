package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

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
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoDoor;
    Servo servoWrist;
    CRServo servoIntake;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;
    double tgtPowerArm = 0;

    double division = 1;

    boolean inputOn = false;
    boolean canInputOn = true;

    @Override
    public void runOpMode() {

        //initialized motors
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");

        //init imu and variables
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

            if (this.gamepad1.dpad_up) {
                tgtPowerArm = 0.2;
            } else if (this.gamepad1.dpad_down) {
                tgtPowerArm = -0.2;
            } else {
                tgtPowerArm = 0;
            }

            if(gamepad1.y) {
                // move to 0 degrees.
                servoWrist.setPosition(0);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoWrist.setPosition(1);
            }

            if (gamepad1.dpad_left) {
                servoDoor.setPosition(0.5);
                sleep(750);
                servoDoor.setPosition(0);
            }

            if (gamepad1.dpad_right) {
                if (canInputOn) {
                    if (inputOn) {
                        inputOn = false;
                        servoIntake.setPower(0);
                    } else {
                        inputOn = true;
                        servoIntake.setPower(0.1);
                    }
                    canInputOn = false;
                }
            } else {
                canInputOn = true;
            }

            tgtPowerForward = (-this.gamepad1.left_stick_y / division);
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division);
            tgtPowerTurn = (this.gamepad1.right_stick_x / division);



            //sets motor power
            setPowerAll(
                    -tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                    tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                    -tgtPowerForward - tgtPowerStrafe - tgtPowerTurn,
                    tgtPowerForward - tgtPowerStrafe - tgtPowerTurn
            );
            armMotor.setPower(tgtPowerArm);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("current wrist amount", servoWrist.getPosition());
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
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoDoor = hardwareMap.get(Servo.class, "door");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoIntake = hardwareMap.get(CRServo.class,"intake");

        servoWrist.setPosition(1);
    }

}