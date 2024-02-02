#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Imu imu(16);

	pros::ADIUltrasonic ultrasonic('a', 'b');
	pros::Distance distance(2);
	pros::ADILineSensor lineSensor('c');
	pros::Gps gps(4);

	imu.reset(true);

	auto orientationSource = Loco::InertialOrientationSource(imu);
	auto posePredictor = Loco::PosePredictor(Eigen::Vector3d(), orientationSource);

	auto particleFilter = Loco::ParticleFilter<100>(orientationSource, posePredictor);
	particleFilter.initializeNormal(Eigen::Vector2d(), Eigen::Matrix2d::Identity() * 0.1);

	auto telemetryRadio = PT::TelemetryRadio(1, new PT::PassThroughEncoding());

	auto fieldModel = Loco::FieldModel({{Eigen::Vector2d(-1.8, -1.8), Eigen::Vector2d(-1.8, 1.8)}, {Eigen::Vector2d(-1.8, 1.8), Eigen::Vector2d(1.8, 1.8)}, {Eigen::Vector2d(1.8, 1.8), Eigen::Vector2d(1.8, -1.8)}, {Eigen::Vector2d(1.8, -1.8), Eigen::Vector2d(-1.8, -1.8)}});

	while (true) {

		Loco::Particle randomParticle = particleFilter.getRandomParticle();

		auto gpsStatus = gps.get_status();

		pros::lcd::print(1, "GPS : %f", gps.get_error());


//		telemetryRadio.transmit("[" + std::to_string(static_cast<int>((randomParticle.getState().x() * 350.0 / 1.8 + 350))) +"," + std::to_string(
//				static_cast<int>((randomParticle.getState().y() * 350.0 / 1.8 + 350))) + "]\n");

//		printf("%s\n", ("[" + std::to_string(static_cast<int>((randomParticle.getState().x() * 350.0 / 1.8 + 350))) +"," + std::to_string(
//				static_cast<int>((randomParticle.getState().y() * 350.0 / 1.8 + 350))) + "]\n").c_str());

		std::cout << "Distance: " << fieldModel.getDistanceToObstacle(Loco::Particle(Eigen::Vector3d(gpsStatus.y, gpsStatus.x, -gpsStatus.yaw * degree.Convert(radian)))).Convert(inch) << ", " << (distance.get() * 1_mm).Convert(inch) << std::endl;

		pros::lcd::print(0, "Model: %f, Reading: %f", fieldModel.getDistanceToObstacle(Loco::Particle(Eigen::Vector3d(gpsStatus.y, gpsStatus.x, -gpsStatus.yaw * degree.Convert(radian)))).Convert(inch), (distance.get() * 1_mm).Convert(inch));
		pros::delay(10);
	}
}
