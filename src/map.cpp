#include "map.hpp"

namespace planner {
    std::ostream& operator << (std::ostream& out, CellType type) {
        return out << static_cast<int>(type);
    }

    std::ostream& operator<<(std::ostream& out, Point point) {
        return out << "Point{ " << point.x << ", " << point.y << " }";
    }

    bool Point::operator==(const Point& p) const {
        return x == p.x && y == p.y;
    }
}