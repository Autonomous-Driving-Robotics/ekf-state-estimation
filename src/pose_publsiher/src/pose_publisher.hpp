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

    Position GetPosition(double delta)
    {
        this->pose.x += delta + this->noise_x(this->generator);
        this->pose.y += delta + this->noise_y(this->generator);
        return this->pose;
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
};
}  // namespace pose_publisher