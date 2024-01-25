#pragma once

#include "locolib/units/units.hpp"

namespace Loco {
	class DeadWheel {
	private:
		QLength lastReading;
		QLength delta;

		QLength radius;
	protected:
		void updateReadingDistance(QLength const newReading) {
			delta = newReading - lastReading;
			lastReading = newReading;
		}

		void updateReading(Angle angle) {
			updateReadingDistance(angle.getValue() * radius.getValue());
		}
	public:
		DeadWheel() = delete;
		explicit DeadWheel(QLength radius) :
			radius(radius),
			lastReading(0.0),
			delta(0.0) {

		}

		virtual void update() {}

		QLength getDisplacement() const {
			return lastReading;
		}

		QLength getDelta() const {
			return delta;
		}
	};
}
