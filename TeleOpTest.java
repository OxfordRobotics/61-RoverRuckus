package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp61Test", group="Iterative Opmode")
@Disabled

public class TeleOpTest extends OpMode {

    HardwareMap hwMap = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    //runs once when driver hits init
    @Override
    public void init() {
        hwMap.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //runs repeatedly when driver hits init
    @Override
    public void init_loop() {

    }

    //runs once when driver hits start
    @Override
    public void start() {
        runtime.reset();
    }

    //runs repeatedly when driver hits start
    @Override
    public void loop() {
        //makes the wheels turn  a half the speed to maintain control
        hwMap.motorLeft.setPower(.5 * gamepad1.left_stick_y);
        hwMap.motorRight.setPower(.5 * gamepad1.right_stick_y);
        hwMap.extendo.setPower(.5 * gamepad2.right_stick_y);
        hwMap.lifto.setPower(.25 * gamepad2.left_stick_y);

        telemetry.addData("left", hwMap.motorLeft.getCurrentPosition());
        telemetry.addData("right", hwMap.motorRight.getCurrentPosition());
        // using set to position. its like encoders.
        if (gamepad1.dpad_up) {

                hwMap.motorLift.setPower(1);

        }
        if (gamepad1.dpad_down) {
            hwMap.motorLift.setPower(-1);
        }
        else{
            hwMap.motorLift.setPower(0);

        }

        //if (gamepad2.a)
        //    hwMap.colServo.setPower(1.0);
        //else
        //   hwMap.colServo.setPower(0);

        //if (gamepad2.dpad_up)
        //    hwMap.bucket.setPosition(Math.min(hwMap.bucket.getPosition() + 0.05, 1));

        //if (gamepad2.dpad_down)
        //    hwMap.bucket.setPosition(Math.max(hwMap.bucket.getPosition() - 0.05, 0));


        telemetry.addData("motorLeft Power: ", hwMap.motorLeft.getPower());
        telemetry.addData("motorRight Power: ", hwMap.motorRight.getPower());

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();
    }

    @Override //runs once when driver hits stop
    public void stop() {

    }
}