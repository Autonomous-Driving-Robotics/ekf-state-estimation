#include <random>

namespace pose_publisher
{
struct Position
{
    double x;
    double y;
};

class DataGenerator
{
  public:
    DataGenerator(double mean_x, double mean_y, double std_x, double std_y)
    {
        this->mean_x = mean_x;
        this->mean_y = mean_y;
        this->std_x = std_x;
        this->std_y = std_y;

        this->noise_x = std::normal_distribution<double>(this->mean_x, this->std_x);
        this->noise_y = std::normal_distribution<double>(this->mean_y, this->std_y);
    }

    Position GetPosition(double delta_t)
    {
        this->pose.x += vx * delta_t + 0.5 * ax * delta_t * delta_t + noise_x(generator);
        vx += ax * delta_t;
        double y{pose.y + noise_y(generator)};
        return Position{this->pose.x, y};
    }

  private:
    Position pose{0.0, 0.0};
    std::default_random_engine generator;
    std::normal_distribution<double> noise_x{};
    std::normal_distribution<double> noise_y{};
    double mean_x{};
    double mean_y{};
    double std_x{};
    double std_y{};

    double vx{0.0};
    double vy{0.0};
    double ax{0.1};
    double ay{0.0};
};
}  // namespace pose_publisher