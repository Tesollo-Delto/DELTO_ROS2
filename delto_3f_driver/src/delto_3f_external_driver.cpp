#include "delto_3f_external_driver.hpp"

DeltoExternalDriver::DeltoExternalDriver() : Node("delto_3f_driver"), publish_rate(500.0)
{
    this->declare_parameter<std::string>("ip", "169.254.186.72");
    this->declare_parameter<int>("port", 10000);
    this->declare_parameter<int>("slaveID", 1);
    this->declare_parameter<bool>("dummy", false);

    //     self.jcm_action_server = ActionServer(
    //     self,
    //     FollowJointTrajectory,
    //     'delto_controller/follow_joint_trajectory',
    //     execute_callback=self.execute_callback,
    //     goal_callback=self.goal_callback,
    //     cancel_callback=self.cancel_callback,

    // )
    // rclcpp::Service<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_server;

    follow_joint_trajectory_server = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "delto_controller/follow_joint_trajectory",
        std::bind(&DeltoExternalDriver::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DeltoExternalDriver::handle_cancel, this, std::placeholders::_1),
        std::bind(&DeltoExternalDriver::handle_accepted, this, std::placeholders::_1));

    kp = std::vector<double>(12, 1.0);
    kd = std::vector<double>(12, 5.0);

    target_joint_state = std::vector<double>(12, 0.0);
    current_joint_state = std::vector<double>(12, 0.0);
    joint_dot = std::vector<double>(12, 0.0);
    pre_joint_state = std::vector<double>(12, 0.0);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    qos.reliable();

    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("gripper/joint_states", qos);

    grasp_sub = this->create_subscription<std_msgs::msg::Int32>("gripper/cmd", qos, [this](const std_msgs::msg::Int32::SharedPtr msg) { /* Callback implementation */ });
    
    grasp_mode_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("gripper/target_joint", qos, std::bind(&DeltoExternalDriver::targetjoint_callback, this, std::placeholders::_1));
    // grasp_mode_sub2= this->create_subscription<ign_msgs::msg::Float>("gripper/target_joint", qos, std::bind(&DeltoExternalDriver::targetjoint_callback, this, std::placeholders::_1));

    joint_state_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)), std::bind(&DeltoExternalDriver::timer_callback, this));

    delto_ip = this->get_parameter("ip").as_string();
    delto_port = this->get_parameter("port").as_int();

    // auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"F1M1", "F1M2", "F1M3", "F1M4",
                        "F2M1", "F2M2", "F2M3", "F2M4",
                        "F3M1", "F3M2", "F3M3", "F3M4"};

    delto_client = std::make_unique<DeltoTCP::Communication>(delto_ip, delto_port);

    delto_client->connect();

    prev_time = this->get_clock()->now();
}

DeltoExternalDriver::~DeltoExternalDriver() {}

// void DeltoExternalDriver::connect()
// {
//     delto_client->connect();
// }

std::vector<double> DeltoExternalDriver::get_position()
{
    return current_joint_state;
}

rclcpp_action::GoalResponse DeltoExternalDriver::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeltoExternalDriver::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DeltoExternalDriver::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DeltoExternalDriver::execute_callback, this, std::placeholders::_1), goal_handle}.detach();
}

void DeltoExternalDriver::execute_callback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Recieved goal");

    auto goal = goal_handle->get_goal();
    
    int index = 0;
    std::vector<std::vector<double>> trajecotry_joint_list;
    // 구간 목표점
    std::vector<double> start_joint_state;
    std::lock_guard<std::mutex> lock(mutex_);

    bool start_joint_flag = true;

    for (auto point : goal->trajectory.points)
    {
        trajecotry_joint_list.push_back(point.positions);
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();


    // Set the result to succes
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

    // Send the result
    goal_handle->succeed(result);

    while (true)
    {
        // std::stringstream ss;
        // auto t = goal->trajectory.points[index].time_from_start;
        // int sec = t.sec;
        // int nsec = t.nanosec;

        // ss << sec << "." << nsec;

        // RCLCPP_INFO(this->get_logger(), "target : %s", ss.str().c_str());

        // trajectory 목표점과 차이가 50% 이내면 다음 인덱스로
        if (index < trajecotry_joint_list.size())
        {
            if (start_joint_flag)
            {
                start_joint_state = current_joint_state;
                start_joint_flag = false;
            }

            target_joint_state = trajecotry_joint_list[index];
            auto diff = 0.0;

            for (int i = 0; i < 12; i++)
            {

                if (target_joint_state[i] - start_joint_state[i] <= 0.0001)
                {
                    diff = 0;

                    break;
                }

                auto temp_diff = std::abs((current_joint_state[i] - target_joint_state[i]) / (target_joint_state[i] - start_joint_state[i]));

                if (temp_diff > diff)
                {
                    diff = temp_diff;
                }
            }

            if (diff < 0.5)
            {
                index++;
                start_joint_flag = true;
            }
        }
        else
        {
            break;
        }
    }
}

void DeltoExternalDriver::execute_callback_test(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Recieved goal");

    auto goal = goal_handle->get_goal();
    auto start_time = this->get_clock()->now().nanoseconds();

    int index = 0;
    std::vector<std::vector<double>> trajecotry_joint_list;
    // 구간 목표점
    std::vector<double> start_joint_state;
    std::lock_guard<std::mutex> lock(mutex_);


    while (true)
    {
        if (index < goal->trajectory.points.size())
        {
            int now_time = this->get_clock()->now().nanoseconds();
            auto priod_time = (now_time - start_time);
            int trajecotry_time = goal->trajectory.points[index].time_from_start.nanosec 
                                + goal->trajectory.points[index].time_from_start.sec * 1000000000;
            
            target_joint_state = goal->trajectory.points[index].positions;
            
            if( priod_time >= trajecotry_time)
            {
                index++;
            }
        }
        else
        {
            //send success
            break;
        }
    }
}

void DeltoExternalDriver::timer_callback()
{

    // timer period
    // auto current_time = this->get_clock()->now();
    // auto dt = (current_time - prev_time).seconds();
    // prev_time = current_time;
    // RCLCPP_INFO(this->get_logger(), "dt: %f", dt);

    std::lock_guard<std::mutex> lock(mutex_);

    DeltoRecievedData recieved_data = delto_client->get_data();

    if (recieved_data.current.size() != 12 || recieved_data.joint.size() != 12)
    {
        RCLCPP_ERROR(this->get_logger(), "Recieved data error");
        return;
    }

    electric_current_state = recieved_data.current;
    current_joint_state = recieved_data.joint;

    // pub joint state
    if (joint_publish_count++ > 5)
    {
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.position = current_joint_state;
        joint_state_pub->publish(joint_state);

        joint_publish_count = 0;
    }

    for (int i = 0; i < 12; ++i)
    {
        joint_dot[i] = current_joint_state[i] - pre_joint_state[i];
        pre_joint_state = current_joint_state;
    }

    auto tq_u = JointControl(target_joint_state, current_joint_state, joint_dot, kp, kd);
    auto duty = Torque2Duty(tq_u);

    std::vector<int> calcduty;

    calcduty.reserve(12);
    
    // duty 40% 이내 

    for (int i = 0; i < 12; i++)
    {
        if (duty[i] > 40)
        {
            duty[i] = 40;
        }
        else if (duty[i] < -40)
        {
            duty[i] = -40;
        }

        calcduty[i] = int(duty[i] * 10.0);
    }

    delto_client->send_duty(calcduty);
}

void DeltoExternalDriver::targetjoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    target_joint_state = std::vector<double>(msg->data.begin(), msg->data.end());
}

std::vector<double> DeltoExternalDriver::JointControl(std::vector<double> target_joint_state,
                                                      std::vector<double> current_joint_state,
                                                      std::vector<double> joint_dot,
                                                      std::vector<double> kp,
                                                      std::vector<double> kd)
{

    std::vector<double> tq_u(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        tq_u[i] = kp[i] * (target_joint_state[i] - current_joint_state[i]) - (kd[i] * joint_dot[i]);
    }
    return tq_u;
}

std::vector<double> DeltoExternalDriver::Torque2Duty(std::vector<double> tq_u)
{

    std::vector<double> duty(12, 0.0);

    for (int i = 0; i < 12; ++i)
    {
        double v = 14.1 / 0.8 * tq_u[i];

        duty[i] = 100.0 * v / 11.1;

        if (duty[i] > 70.0)
        {
            duty[i] = 70.0;
        }
        else if (duty[i] < -70.0)
        {
            duty[i] = -70.0;
        }
    }

    return duty;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DeltoExternalDriver>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
