#include <rclcpp/rclcpp.hpp>

// include headers for synchronized callbacks
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>

// include custom messages
#include <leg_detector_msgs/msg/person_array.hpp>
#include <leg_detector_msgs/msg/person.hpp>

// include header for sensor_msgs LaserScan
#include <sensor_msgs/msg/laser_scan.hpp>

// include header for performing linear algebra calculations
#include <eigen3/Eigen/Dense>

class InflatedHumanScanNode : public rclcpp::Node
{

    public:

        InflatedHumanScanNode(): Node("inflated_human_scan"),
                             scan_sub_(this, "scan", rclcpp::SensorDataQoS()),
                             people_tracked_sub_(this, "people_tracked", rclcpp::SystemDefaultsQoS()),
                             sync_(10, scan_sub_, people_tracked_sub_)
                                                            
        {
            // get the inflation radius parameter
            this->declare_parameter("scan_topic", "/scan");
            this->get_parameter_or("scan_topic", scan_topic_, std::string("/scan"));
            this->declare_parameter("people_tracked_topic", "people_tracked");
            this->get_parameter_or("people_tracked_topic", people_tracked_topic_, std::string("people_tracked"));
            this->declare_parameter("inflation_radius", 1.0);
            this->get_parameter_or("inflation_radius", inflation_r, 1.0);
            RCLCPP_INFO(this->get_logger(), "%f", inflation_r);

            // subscribe to the scan topic a(nd people tracked topic (for future design changes)
            scan_sub_.subscribe(this, scan_topic_, rclcpp::SensorDataQoS());
            people_tracked_sub_.subscribe(this, people_tracked_topic_, rclcpp::SystemDefaultsQoS());

            // register a synchronized callback
            //message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, interfaces::msg::PersonArray> sync_(scan_sub_, people_tracked_sub_, 200);
            sync_.registerCallback(std::bind(&InflatedHumanScanNode::inflated_human_callback, this, std::placeholders::_1, std::placeholders::_2));

            // publish to the inflated_human topic
            ihs_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("inflated_human_scan", rclcpp::SensorDataQoS());
        }

    private:

        // create 2 subscribers
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
        message_filters::Subscriber<leg_detector_msgs::msg::PersonArray> people_tracked_sub_;
        message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, leg_detector_msgs::msg::PersonArray> sync_;

        // create one publisher
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ihs_pub_;

        // private variables
        float angle_min;
        float angle_max;
        float angle_inc;
        double inflation_r;
        std::string scan_topic_;
        std::string people_tracked_topic_;

        rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &params)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          result.reason = "success";

          for (const auto &param : params)
          {
            if (param.get_name() == "scan_topic")
            {
              if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !param.as_string().empty())
              {
                RCLCPP_INFO(this->get_logger(), "Updated scan_topic to %s", param.as_string().c_str());
                scan_topic_ = param.as_string();
              }
              else
              {
                result.successful = false;
                result.reason = "scan_topic can not be empty";
              }
            }
            else if (param.get_name() == "people_tracked_topic")
            {
              if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !param.as_string().empty())
              {
                RCLCPP_INFO(this->get_logger(), "Updated people_tracked_topic to %s", param.as_string().c_str());
                people_tracked_topic_ = param.as_string();
              }
              else
              {
                result.successful = false;
                result.reason = "people_tracked_topic can not be empty";
              }
            }
            else if (param.get_name() == "inflation_radius")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && param.as_double() > 0.0)
                {
                    RCLCPP_INFO(this->get_logger(), "Updated inflation_radius to %.2f", param.as_double());
                    inflation_r = param.as_double();
                }
                else
                {
                    result.successful = false;
                    result.reason = "inflation_radius must be > 0";
                }
            }
          }
          return result;
        }
        // define the callback function
        void inflated_human_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan, const leg_detector_msgs::msg::PersonArray::ConstSharedPtr &people_tracked) {
            sensor_msgs::msg::LaserScan updated_human_scan_ = sensor_msgs::msg::LaserScan();
            // getting scan parameters
            angle_min = scan->angle_min;
            angle_max = scan->angle_max;
            angle_inc = scan->angle_increment;

            // initialize the human scans to subscribed laser scan topic
            updated_human_scan_.ranges = scan->ranges;

            for (long unsigned int i = 0; i < people_tracked->people.size(); i++)
            {
                float xH = people_tracked->people[i].pose.position.x;
                float yH = people_tracked->people[i].pose.position.y;
                float dH = sqrt(xH*xH + yH*yH);

                //call function to return a scan with inflated radius
                if (dH > inflation_r)
                    inflate_human_position(xH, yH, &updated_human_scan_);
            }

            // update the other data in updated_human_scan topic
            updated_human_scan_.header = scan->header;
            updated_human_scan_.angle_increment = scan->angle_increment;
            updated_human_scan_.angle_max = scan->angle_max;
            updated_human_scan_.angle_min = scan->angle_min;
            updated_human_scan_.intensities = scan->intensities;
            updated_human_scan_.range_max = scan->range_max;
            updated_human_scan_.range_min = scan->range_min;
            updated_human_scan_.scan_time = scan->scan_time;
            updated_human_scan_.time_increment = scan->time_increment;

            // publish the updated laser scan
            ihs_pub_->publish(updated_human_scan_);
        }

        // function that generages a bunch of points around a tracked human incorporating the inflation radius
        void inflate_human_position (float xH, float yH, sensor_msgs::msg::LaserScan *updated_human_scan) {

            Eigen::VectorXf temp_ranges;
            Eigen::VectorXf temp_angles;
            Eigen::VectorXi temp_ind;

            // Derivations for calculating the ranges[] values
            float dH = sqrt(xH*xH + yH*yH);
            float angle = atan2(yH,xH);
            float theta_tangent = asin(inflation_r/dH);

            // Calculating the distance of the circle at different angles
            Eigen::ArrayXf delta_theta = Eigen::VectorXf::LinSpaced(floor(2*theta_tangent/angle_inc),-theta_tangent, theta_tangent);
            temp_ranges = dH*delta_theta.cos() - (inflation_r*inflation_r - dH*dH*(delta_theta.sin()).square()).sqrt();
            temp_angles = angle + delta_theta;
            temp_ind = ((temp_angles.array() - angle_min)/angle_inc).cast<int>();

            // Updating the new scan topic with new ranges[] values
            for (int i = 0; i < temp_ind.size(); i++)
            {
                if (temp_ind[i] > 511)
                    temp_ind[i] = 510;
                else if (temp_ranges[i] < 1)
                    temp_ranges[i] = 1;
                
                if (updated_human_scan->ranges[temp_ind(i)] > temp_ranges(i))
                    updated_human_scan->ranges[temp_ind(i)] = temp_ranges(i);
            }

        }
};

int main(int argc , char **argv) {

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<InflatedHumanScanNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
