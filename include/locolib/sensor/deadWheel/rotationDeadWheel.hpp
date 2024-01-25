#pragma once

#include "deadWheel.hpp"
#include "../../../pros/rotation.hpp"

namespace Loco {
	class RotationDeadWheel : public DeadWheel {
	private:
		pros::Rotation& rotation;
	public:
		RotationDeadWheel() = delete;
		RotationDeadWheel(pros::Rotation& rotation, QLength radius) : DeadWheel(radius), rotation(rotation) {}

		void update() final {
			DeadWheel::updateReading(rotation.get_position() * 0.01_deg);
		}
	};
}