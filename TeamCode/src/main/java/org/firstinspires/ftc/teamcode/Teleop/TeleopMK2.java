package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleopMK2 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double tgtPowerForward = 0;
    double tgtPowerStrafe = 0;
    double tgtPowerTurn = 0;

    double debounceCount = 0;

    @Override
    public void runOpMode() {

        //initialized motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            /*this cuts the target power in half if you're
            holding down the b button on the controller*/
            if (!this.gamepad1.b) {
                tgtPowerForward = this.gamepad1.left_stick_y;
                tgtPowerStrafe = this.gamepad1.left_stick_x;
                tgtPowerTurn = this.gamepad1.right_stick_x;
            } else {
                tgtPowerForward = (this.gamepad1.left_stick_y / 2);
                tgtPowerStrafe = (this.gamepad1.left_stick_x / 2);
                tgtPowerTurn = (this.gamepad1.right_stick_x / 2);
            }

            //sets motor power
            frontLeft.setPower(tgtPowerForward - tgtPowerStrafe - tgtPowerTurn);
            backLeft.setPower(tgtPowerForward + tgtPowerStrafe - tgtPowerTurn);
            frontRight.setPower(-tgtPowerForward - tgtPowerStrafe - tgtPowerTurn);
            backRight.setPower(-tgtPowerForward + tgtPowerStrafe - tgtPowerTurn);

            telemetry.addData("Debounce", debounceCount);
            telemetry.update();
        }
    }
}