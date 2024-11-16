#include "formation_comm.h"

/*
 * 所有智能体的位姿订阅回调函数
 * msg: 位姿话题的消息指针
 * id:  机器人唯一id号
 */
void FormationCOMM::agentsPoseCb(const nav_msgs::Odometry::ConstPtr& msg, size_t agent_id)
{
    /* 更新标识符为agent_id机器人的属性表 */
    tf::Quaternion quat;
    double roll, pitch, yaw;
    RobotAttributes& attr = attrs.at(agent_id);
    attr.state.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    attr.state.vel << msg->twist.twist.linear.x, msg->twist.twist.angular.z;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    attr.state.heading = yaw;

    agents.at(agent_id).attributes = attr;  /* 同步更新信息表 */
}

FormationCOMM& FormationCOMM::get_instance(ros::NodeHandle& handle, const std::vector<RobotTopic>& topics,
                                           int my_id, int loop_hz)
{
    static FormationCOMM instance(handle, topics, my_id, loop_hz);
    return instance;
}

/*
 * 构造函数，订阅和建立话题通信
 * nh: ROS节点句柄
 * topics: 速度和位置主题名称
*/
FormationCOMM::FormationCOMM(ros::NodeHandle& handle, const std::vector<RobotTopic>& topics, 
                             int my_id, int loop_hz) : nh(handle)
{
    if (!FormationCOMM::checkUniqueTopics(topics)) {
        ros::shutdown();
        return;
    }
    
    id  = my_id;
    hz  = loop_hz;
    num = topics.size();
    cnts.reserve(num);
    attrs.reserve(num);
    agents.reserve(num);

    for (size_t i = 0; i < num; ++i) {
        const RobotTopic& topic = topics.at(i);
        int agent_id = topic.info.id;

        /* 初始化信息表 */
        RobotSystem agent;
        agent.attributes.info = topic.info;
        agent.pose_topic.name = topic.name.pos;
        agent.vel_topic.name  = topic.name.vel;
        FormationCOMM::waitForTopic<nav_msgs::Odometry>(topic.name.pos);
        agent.pose_topic.suber = nh.subscribe<nav_msgs::Odometry>(topic.name.pos,           /* 所有智能体的位姿订阅话题 */
                        5, boost::bind(&FormationCOMM::agentsPoseCb, this, _1, agent_id));
        if (agent_id == id)
            agent.vel_topic.puber = nh.advertise<geometry_msgs::Twist>(topic.name.vel, 1);  /* 本智能体的速度控制发布话题 */
        agents.insert(std::make_pair(agent_id, agent));

        cnts.insert(std::make_pair(agent_id, 0));
        attrs.insert(std::make_pair(agent_id, RobotAttributes()));
    }
}

FormationCOMM::~FormationCOMM() 
{
    ROS_INFO("FormationCOMM destructor called!");
}

/* 返回智能体的状态信息 */
std::unordered_map<int, RobotAttributes> FormationCOMM::receiveRobotState(void)
{
    /* 计算属性表可信度 */
    for (int i = 0; i < num; ++i) { 
        attrs.at(i).state.credibility = cnts.at(i) / (50.0 / hz);
        cnts.at(i) = 0; 
    }

    return attrs;
}

/*
 * 向本智能体发送速度指令
 * vel: 速度信息，包含线速度和角速度
 * return true: 如果发送成功
 * return false: 如果发送失败或ID无效
*/
bool FormationCOMM::sendVelInfo(const Eigen::Vector2d& vel) const
{
    bool result = false;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x  = vel[0];
    vel_msg.angular.z = vel[1];

    try {
        agents.at(id).vel_topic.puber.publish(vel_msg);
        result = true;
    }
    catch(const std::out_of_range&) {
        ROS_ERROR("FormationCOMM::sendVelInfo内部错误:未能找到匹配的机器人id!");
    }

    return result;
}

/* 等待指定的话题可用 */
template<typename T>
bool FormationCOMM::waitForTopic(const std::string& topic, double timeout)
{
    boost::shared_ptr<T const> msg;
    msg = ros::topic::waitForMessage<T>(topic, ros::Duration(timeout));
    if(msg == nullptr) {
        ROS_ERROR("订阅话题超时：%s", topic.c_str());
        ros::shutdown();
        return false;
    }

    return true;
}

/* id和name的重复性检查 */
bool FormationCOMM::checkUniqueTopics(const std::vector<RobotTopic>& topics) {
    std::unordered_set<int> seen_ids;
    std::unordered_set<std::string> seen_names;
    seen_ids.reserve(num);
    seen_names.reserve(num);

    for (const auto& topic : topics) {
        if (!seen_ids.insert(topic.info.id).second) {
            ROS_ERROR("发现重复的机器人id: %d", topic.info.id);
            return false;
        }
        if (!seen_names.insert(topic.info.name).second) {
            ROS_ERROR("发现重复的机器人name: %s", topic.info.name.c_str());
            return false;
        }
    }

    return true;
}


