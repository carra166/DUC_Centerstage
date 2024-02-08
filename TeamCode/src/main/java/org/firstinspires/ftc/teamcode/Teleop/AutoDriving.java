@ -1,254 +0,0 @@
package org.firstinspires.ftc.teamcode.Teleop;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
10430 TELEOP
Motor gamepad controls
*/
@TeleOp
@Config
public class AutoDriving extends LinearOpMode {

    private static final String BASE_FOLDER_NAME = "autonomousTexts";
    public static String textFileName = "test";

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    IMU imu;
    YawPitchRollAngles robotOrientation;

    Servo servoClaw;
    Servo servoWrist;
    Servo servoLauncher;
    Servo servoPooper;

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

    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;

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

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoWrist.setPosition(0.1);
        servoClaw.setPosition(0.5);
        servoLauncher.setPosition(0);
        servoPooper.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (this.gamepad1.right_trigger > 0) {
                //slow
                division = 5;
            } else if (this.gamepad1.left_trigger > 0) {
                //fast
                division = 2;
            } else {
                //normal
                division = 3.5;
            }

            //write the encoder values to the text file
            if (gamepad1.x) {
                while (gamepad1.x) {}
                try {
                    FileWriter writer = new FileWriter(directoryPath+"/"+textFileName+".txt", true); // Appending mode
                    double[] numbers = {frontRight.getCurrentPosition(), frontLeft.getCurrentPosition(), backRight.getCurrentPosition(), backLeft.getCurrentPosition()};
                    StringBuilder sb = new StringBuilder();
                    for (double num : numbers) {
                        sb.append(Double.toString(num)).append(" "); // Converting doubles to strings and appending
                    }
                    writer.write(sb.toString().trim() + "\n"); // Appending a new line with the formatted string
                    writer.close();

                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                } catch (IOException e) {
                    System.out.println("An error occurred.");
                    e.printStackTrace();
                }
            }

            //clear text file
            if (gamepad1.b) {
                while (gamepad1.b) {}
                try {
                    FileWriter writer = new FileWriter(directoryPath+"/"+textFileName+".txt"); // Appending mode
                    writer.write(""); // Appending a new line with the formatted string
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            //pooper function call
            if (gamepad1.y) {
                while (gamepad1.y) {}
                servoPooper.setPosition(0.5);
                try {
                    FileWriter writer = new FileWriter(directoryPath+"/"+textFileName+".txt", true);
                    writer.write("pooper\n");
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            //backboard placement function call
            if (gamepad1.a) {
                while (gamepad1.a) {}
                try {
                    FileWriter writer = new FileWriter(directoryPath+"/"+textFileName+".txt", true);
                    writer.write("backboard\n");
                    writer.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            tgtPowerForward = (-this.gamepad1.left_stick_y / division);
            tgtPowerStrafe = (-this.gamepad1.left_stick_x / division);
            tgtPowerTurn = (this.gamepad1.right_stick_x / division);
            tgtPowerArm = ((this.gamepad2.left_stick_y / -5) + calculateArmPower(armAngles.get(armMotor.getCurrentPosition()), kCos) + downPower);

            //sets motor powereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew44444444444444    q1`1`
            //this comment corrupted but its just too funny to delete
            setPowerAll(
                        -tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward + tgtPowerStrafe - tgtPowerTurn,
                        -tgtPowerForward - tgtPowerStrafe - tgtPowerTurn,
                        tgtPowerForward - tgtPowerStrafe - tgtPowerTurn
            );
            /*armMotor.setPower(tgtPowerArm);
            armMotor2.setPower(-tgtPowerArm);*/

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("front right wheel encoder position", frontRight.getCurrentPosition());
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

        servoClaw = hardwareMap.get(Servo.class, "claw");
        servoWrist = hardwareMap.get(Servo.class, "wrist");
        servoLauncher = hardwareMap.get(Servo.class, "launcher");
        servoPooper = hardwareMap.get(Servo.class, "pooper");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private double calculateArmPower(double armAngle, double kCos) {
        return kCos * Math.cos(Math.toRadians(armAngle));
    }

}