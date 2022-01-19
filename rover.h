#ifndef JNP6_ROVER_H
#define JNP6_ROVER_H

#include <utility>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <initializer_list>

using coordinate_t = int32_t;
using direction_t = std::pair<int, int>;

class RoverNotLanded : public std::exception {
public:
    [[nodiscard]] inline const char *what() const noexcept override {
        return "RoverNotLanded";
    }
};

struct Direction {
    static constexpr auto NORTH = std::make_pair(0, 1);
    static constexpr auto EAST = std::make_pair(1, 0);
    static constexpr auto SOUTH = std::make_pair(0, -1);
    static constexpr auto WEST = std::make_pair(-1, 0);

    static std::string getName(direction_t direction) {
        if (direction == NORTH)
            return "NORTH";
        if (direction == EAST)
            return "EAST";
        if (direction == SOUTH)
            return "SOUTH";
        if (direction == WEST)
            return "WEST";

        return "unknown";
    }
};

class Rover;
class State;

class Operation {
public:
    virtual ~Operation() = default;

    virtual void execute(Rover *rover) const = 0;
};

class Sensor {
public:
    virtual bool is_safe([[maybe_unused]] coordinate_t x,
                         [[maybe_unused]] coordinate_t y) = 0;
};

class State {
private:
    coordinate_t x{};
    coordinate_t y{};
    direction_t direction{};
    bool stopped{};

public:
    State() = default;

    State(coordinate_t x, coordinate_t y, direction_t &direction, bool stopped)
        : x(x), y(y), direction(direction), stopped(stopped) {}

    [[nodiscard]] coordinate_t getX() const {
        return x;
    }

    [[nodiscard]] coordinate_t getY() const {
        return y;
    }

    [[nodiscard]] direction_t getDirection() const {
        return direction;
    }

    [[nodiscard]] bool isStopped() const {
        return stopped;
    }

    void setStopped(bool s) {
        stopped = s;
    }
};

using Operations_map = std::unordered_map<char, std::shared_ptr<Operation>>;
using Sensor_vec = std::vector<std::shared_ptr<Sensor>>;

class Rover {
private:
    Sensor_vec sensors;
    std::shared_ptr<State> state;
    Operations_map operations;
    bool landed = false;

public:
    Rover(Operations_map &operationsMap, Sensor_vec &sensorVec) {
        sensors = sensorVec;
        operations = operationsMap;
        state = std::make_shared<State>();
    }

    void land(std::pair<int, int> coordinates, direction_t direction) {
        landed = true;

        coordinate_t x = coordinates.first;
        coordinate_t y = coordinates.second;

        auto new_state = std::make_shared<State>(x, y, direction, false);
        setState(new_state);
    }

    void execute(const std::string &commands) {
        if (!landed)
            throw RoverNotLanded();

        state->setStopped(false);

        for (char key : commands) {
            if (!operations.contains(key)) {
                state->setStopped(true);
                break;
            }
            operations[key]->execute(this);
        }
    }

    bool danger_exists(coordinate_t new_x, coordinate_t new_y) {
        for (auto &sensor_ptr : sensors) {
            if (!sensor_ptr->is_safe(new_x, new_y))
                return true;
        }
        return false;
    }

    friend std::ostream &operator<<(std::ostream &output, const Rover &r) {
        std::string name = Direction::getName(r.getState()->getDirection());

        if (name == "unknown") {
            output << name;
            return output;
        }

        output << "(" << r.getState()->getX() << ", " << r.getState()->getY()
               << ") " << name;
        if (r.getState()->isStopped()) {
            output << " stopped";
        }

        return output;
    }

    const std::shared_ptr<State> &getState() const {
        return state;
    }

    void setState(const std::shared_ptr<State> &s) {
        state = s;
    }
};

class RoverBuilder {
private:
    Operations_map builder_operations;
    Sensor_vec builder_sensors;
public:
    RoverBuilder() : builder_operations(), builder_sensors() {}

    RoverBuilder program_command(char key, std::shared_ptr<Operation> op) {
        builder_operations[key] = std::move(op);
        return *this;
    }

    RoverBuilder add_sensor(std::unique_ptr<Sensor> sensor) {
        builder_sensors.push_back(std::move(sensor));
        return *this;
    }

    Rover build() {
        return Rover(builder_operations, builder_sensors);
    }
};

class Compose : public Operation {
private:
    std::vector<std::shared_ptr<Operation>> operations;

public:
    Compose(std::initializer_list<std::shared_ptr<Operation>> ops)
        : operations(ops) {}

    void execute(Rover *rover) const override {
        for (auto &operation : operations) {
            if (rover->getState()->isStopped())
                break;
            operation->execute(rover);
        }
    }
};

class Move : public Operation {
public:
    void execute(Rover *rover) const override = 0;

    static void execute_move(Rover *rover, bool forward) {
        direction_t direction = rover->getState()->getDirection();
        coordinate_t new_x, new_y;
        if (forward) {
            new_x = rover->getState()->getX() + direction.first;
            new_y = rover->getState()->getY() + direction.second;
        } else {
            new_x = rover->getState()->getX() - direction.first;
            new_y = rover->getState()->getY() - direction.second;
        }

        bool stopped = false;

        if (rover->danger_exists(new_x, new_y)) {
            stopped = true;
            new_x = rover->getState()->getX();
            new_y = rover->getState()->getY();
        }

        auto new_state =
            std::make_shared<State>(new_x, new_y, direction, stopped);
        rover->setState(new_state);
    }
};

class MoveForward : public Move {
public:
    void execute(Rover *rover) const override {
        Move::execute_move(rover, true);
    }
};

class MoveBackward : public Move {
public:
    void execute(Rover *rover) const override {
        Move::execute_move(rover, false);
    }
};

class Rotate : public Operation {
public:
    void execute(Rover *rover) const override = 0;

    static void execute_rotate(Rover *rover, bool clockwise) {
        coordinate_t new_x = rover->getState()->getX();
        coordinate_t new_y = rover->getState()->getY();
        bool stopped = false;

        direction_t d = rover->getState()->getDirection();
        direction_t direction;
        if (clockwise)
            direction = std::make_pair(d.second, -d.first);
        else
            direction = std::make_pair(-d.second, d.first);

        auto new_state =
            std::make_shared<State>(new_x, new_y, direction, stopped);
        rover->setState(new_state);
    }
};

class RotateLeft : public Rotate {
public:
    void execute(Rover *rover) const override {
        Rotate::execute_rotate(rover, false);
    }
};

class RotateRight : public Rotate {
public:
    void execute(Rover *rover) const override {
        Rotate::execute_rotate(rover, true);
    }
};

std::shared_ptr<MoveForward> move_forward() {
    return std::make_shared<MoveForward>();
}

std::shared_ptr<MoveBackward> move_backward() {
    return std::make_shared<MoveBackward>();
}

std::shared_ptr<RotateLeft> rotate_left() {
    return std::make_shared<RotateLeft>();
}

std::shared_ptr<RotateRight> rotate_right() {
    return std::make_shared<RotateRight>();
}

std::shared_ptr<Compose>
compose(std::initializer_list<std::shared_ptr<Operation>> ops) {
    return std::make_shared<Compose>(ops);
}

#endif // JNP6_ROVER_H

