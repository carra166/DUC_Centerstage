package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopMK1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        //initialized motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            frontLeft.setPower(this.gamepad1.left_stick_y);
            backLeft.setPower(this.gamepad1.left_stick_y);

            frontRight.setPower(-this.gamepad1.right_stick_y);
            backRight.setPower(-this.gamepad1.right_stick_y);

            System.out.println(gamepad1);

            telemetry.addData("Gamepad", this.gamepad1);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}