


@TeleOp(name = "Turret Tuner")
public class turretTuner extends OpMode{
    
    DcMotorEx motorChain;
    DcMotorEx motorBevel;
    double lastTime;

    @Override
    public void init() {
        // Initialize the turretPIDF
        DcMotorEx motorChain = hardwareMap.get(DcMotorEx.class, "topLauncher");
        DcMotorEx motorBevel = hardwareMap.get(DcMotorEx.class, "sideLauncher");
        tuner = new turretPIDF(motorChain, motorBevel);
        lastTime = getRuntime();
    }

    @Override
    public void loop() {

        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // Prevent bad dt values
        if (dt <= 0 || dt > 0.1) return;

        // Joystick input (-1 to 1)
        double joystick = gamepad1.right_stick_x;

        // Max turret angular velocity (rad/s)
        double maxOmega = 3.0;

        // Target angular velocity
        double targetOmega = joystick * maxOmega;

        // Update velocity controller
        tuner.update(targetOmega, dt);
    }

}
