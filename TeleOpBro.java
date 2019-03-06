package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="TeleOp61", group="Iterative Opmode")
public class TeleOpBro extends OpMode {

    HardwareMap hwMap = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    //runs once when driver hits init
    @Override
    public void init() {
        hwMap.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hwMap.idolServo.setPosition(0);
    }

    //runs once when driver hits start
    @Override
    public void start() {
        runtime.reset();
    }

    //runs repeatedly when driver hits start
    double pos = 0;
    @Override
    public void loop() {
        double powerratio = .8;
        if (gamepad1.left_bumper) {
            powerratio = .3;
        }
        hwMap.motorLeft.setPower(powerratio * gamepad1.left_stick_y);
        hwMap.motorRight.setPower(powerratio * gamepad1.right_stick_y);
        hwMap.extendo.setPower(.5 * gamepad2.right_stick_y);

        if (gamepad1.right_trigger > 0) {
            hwMap.motorLift.setPower(-gamepad1.right_trigger);
        }

        if (gamepad1.left_trigger > 0) {
            hwMap.motorLift.setPower(gamepad1.left_trigger);
        }
        else {
            hwMap.motorLift.setPower(0);
        }
        if (gamepad2.a) {
            hwMap.collecto.setPower(-.15);
        }
        else if (gamepad2.b) {
            hwMap.collecto.setPower(-.75);
        }
        else {
            hwMap.collecto.setPower(0);
        }
        if (gamepad2.dpad_up) {
            hwMap.bucket.setPower(.7);
        }
        else if (gamepad2.dpad_down) {
            hwMap.bucket.setPower(-.7);
        }
        else {
            hwMap.bucket.setPower(0);
        }

        if (hwMap.lifto.getCurrentPosition() < 1680 * (2 / 3.0)) {
            hwMap.lifto.setPower(gamepad2.left_stick_y);
        } else if (hwMap.lifto.getCurrentPosition() >= 1680 && gamepad2.left_stick_y >= 0) {
            hwMap.lifto.setPower(0);
        } else if (hwMap.lifto.getCurrentPosition() <= 0 && gamepad2.left_stick_y <= 0) {

        } else {
            hwMap.lifto.setPower(gamepad2.left_stick_y * ((1680 - hwMap.lifto.getCurrentPosition()) / 1680));
        }

        if ( gamepad1.dpad_up ) {
            hwMap.idolServo.setPosition( hwMap.idolServo.getPosition() + 0.01 );
        } else if ( gamepad1.dpad_down ) {
            hwMap.idolServo.setPosition( hwMap.idolServo.getPosition() - 0.01 );
        }

        telemetry.addData("motorLeft Power: ", hwMap.motorLeft.getPower());
        telemetry.addData("motorRight Power: ", hwMap.motorRight.getPower());
        telemetry.addData("Lift Position: ",hwMap.motorLift.getCurrentPosition());
        telemetry.addData("Lifto Position", hwMap.lifto.getCurrentPosition());
        telemetry.addData("servo pos", hwMap.idolServo.getPosition() );

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();
    }

    @Override //runs once when driver hits stop
    public void stop() {

    }
}
// Signed by Valentino Antonio Pirocchi Tm