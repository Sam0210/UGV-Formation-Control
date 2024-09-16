#include "formation_comm.h"


/**
 * @brief FormationCOMM 构造函数，初始化节点和话题。
 * 
 * @param nh ROS节点句柄。
 * @param topics 速度和位置主题名称。
 */
FormationCOMM::FormationCOMM(ros::NodeHandle& nh, const std::vector<RobotTopic>& topics, int myid, int loop_hz) : nh_(nh)
{
    // 机器人id,name的重复性检查
    if (!FormationCOMM::checkUniqueTopics(topics)) {
        ros::shutdown();
        return;
    }
    
    myid_ = myid;
    loop_hz_ = loop_hz;

    // 分配容量
    agent_num_ = topics.size();
    agents_.resize(agent_num_);    // 容器
    idCount_map_.reserve(agent_num_);  // 哈希表
    idAgent_map_.reserve(agent_num_);    // 哈希表  // FIXME: 不支持运行中增加编队成员
    // 调整 id->states(RobotAttributes) 哈希表容量，和建立哈希表的键，供receiveRobotState使用．
        //　在此处调整容量可以避免receiveRobotState频繁的reserve，是耗时的，其时间复杂度O(n)
        // 提前建立键，避免频繁insert带来的错误（在下面for循环中）
    idState_map_.reserve(agent_num_);  // 哈希表

    // TODO: size_t 选择足够大的无符号整形来代表该平台上最大可能出现的对象大小
        // 马上就知道它代表字节大小或数组索引
    for (size_t i=0; i<agent_num_; ++i) {
        RobotSystem& agent = agents_.at(i);
        const RobotTopic& topic = topics.at(i);

        agent.attributes.info = topic.info;      // 机器人id和name
        agent.pose_topic.name = topic.name.pos;  // 机器人位姿话题
        agent.vel_topic.name = topic.name.vel;   // 机器人速度话题

        // 位姿话题订阅  // FIXME:缓存区大小对回调函数执行频率有较大影响．但又与我期望的处理时序不同，需要更该为不同进程
        FormationCOMM::waitForTopic<nav_msgs::Odometry>(agent.pose_topic.name);   // 等待目标话题出现
        agent.pose_topic.suber = nh_.subscribe<nav_msgs::Odometry>(agent.pose_topic.name, 
            5, boost::bind(&FormationCOMM::agentsPoseCb, this, _1, agent.attributes.info.id));

        // 速度话题发布  // TODO:仅发布自己的速度话题
        if (agent.attributes.info.id == myid_)
            agent.vel_topic.puber = nh_.advertise<geometry_msgs::Twist>(agent.vel_topic.name, 1);

        // 建立id->agents_(RobotSystem)和name->agents_(RobotSystem)的哈希表
        idAgent_map_.insert(std::make_pair(agent.attributes.info.id, std::ref(agent)));
        // TODO: map["key"] uses the unordered maps operator[]. 
            // That operator returns a reference to the item in the map. 
            // Which is problematic if there is no such item.
        // idAgent_map_[agent.attributes.info.id] = std::ref(agent);
        // namemap_[agent.attributes.info.name] = std::ref(agent);

        // 提前建立键
        idState_map_.insert(std::make_pair(agent.attributes.info.id, RobotAttributes()));
        idCount_map_.insert(std::make_pair(agent.attributes.info.id, 0));
    }
}

FormationCOMM::~FormationCOMM() 
{
    ROS_INFO("FormationCOMM destructor called!");
}


FormationCOMM& FormationCOMM::get_instance(ros::NodeHandle& nh, const std::vector<RobotTopic>& topics, int myid, int loop_hz)
{
    static FormationCOMM instance(nh, topics, myid, loop_hz);
    return instance;
}


/**
 * @brief 设置领导者和跟随者的主题。
 * 
 * @param leader_topics 领导者的主题名称。
 * @param follower_topics 跟随者的主题名称列表。
 */
void FormationCOMM::setTopics(const std::vector<RobotTopic>& topics)
{
    // balabala
}

/**
 * @brief 获取当前设置的领导者和跟随者的主题。
 * 
 * @return std::pair<TopicName, std::vector<TopicName>> 当前主题设置。
 */
std::vector<RobotTopic> FormationCOMM::getTopics(void) const 
{
    // balabala
}

/**
 * @brief 接收并返回领导者和跟随者的状态信息。
 * 
 * @return std::pair<RobotState, std::vector<RobotState>> 领导者和跟随者的状态信息。
 */
// 以 RobotInfo.id 作为键，直接访问相应的 RobotAttributes
std::unordered_map<int, RobotAttributes> FormationCOMM::receiveRobotState(void)
{
    for (auto& agent : agents_) {
        // 计算可信度（实际频率／理想频率）
        agent.attributes.state.credibility = idCount_map_.at(agent.attributes.info.id) / (50.0 / loop_hz_); 
        idCount_map_.at(agent.attributes.info.id) = 0;

        idState_map_.at(agent.attributes.info.id) = agent.attributes; 
    }

    return idState_map_;
}


/**
 * @brief 发送速度信息到指定的机器人。
 * 
 * 这个函数向指定的机器人发送速度命令。速度信息包含线速度和角速度。
 * 
 * @param id 机器人唯一标识符，1～ｎ。
 * @param vel 速度信息，包含线速度和角速度。
 * @return true 如果发送成功。
 * @return false 如果发送失败或ID无效。
 */
bool FormationCOMM::sendVelInfo(const Eigen::Vector2d& vel) const
{
/* 方案1：通过find_if查找与id匹配的机器人，时间复杂度为O(n)
    // 与机器人id进行匹配
    // TODO:整理std::find_if知识点
    // find_if函数 带条件的查找元素，可以通过find_if函数来实现查找满足特定条件的元素。
        // find_if函数依次的遍历容器的元素，返回第一个使函数为true的元素的迭代器，如果查找失败则返回end迭代器。
        // 它与find函数的区别就是不用重写==，而是自己定义查找条件。
    auto it = std::find_if(agents_.begin(), agents_.end(), [id](const RobotSystem& candidate_agent) {  // FIXME:使用频率高，但O(n)
        return candidate_agent.attributes.info.id == id;
    });

    // TODO: 整理
    // 最后一个元素的迭代器不是 .end()。
        // .end() 实际上是指向 vector 末尾元素的下一个位置的迭代器，这是一个特殊的迭代器，
        // 用于表示 vector 的“尾后”（past-the-end）位置。
    if(it == agents_.end()) {  
        ROS_WARN("FormationCOMM::sendVelInfo:未能找到与输入id匹配的机器人id!");
        return false;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x  = vel[0];
    vel_msg.angular.z = vel[1];

    const RobotSystem& the_agent = *it;
    the_agent.vel_topic.puber.publish(vel_msg);

    return true;
*/
    bool result = false;

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x  = vel[0];
    vel_msg.angular.z = vel[1];

    try {
        RobotSystem& me_agent = idAgent_map_.at(myid_).get();
        me_agent.vel_topic.puber.publish(vel_msg);  //TODO: 测试ROS日志是否会报错
        result = true;
    }
    catch(const std::out_of_range&) {
        ROS_ERROR("FormationCOMM::sendVelInfo内部错误:未能找到匹配的机器人id!");
    }

    return result;
}

/**
 * @brief 等待指定的话题可用。
 * 
 * @tparam T 消息类型。
 * @param topic 要等待的话题名称。
 * @param timeout 等待超时时间，默认是5秒。
 * @return true 如果话题可用。
 * @return false 如果在超时时间内话题不可用。
 */
template<typename T>
bool FormationCOMM::waitForTopic(const std::string& topic, double timeout)
{
    boost::shared_ptr<T const> msg;  // new writing style
    msg = ros::topic::waitForMessage<T>(topic, ros::Duration(timeout));
    
    if(msg == nullptr) {
        ROS_ERROR("未能订阅话题：%s", topic.c_str());
        ros::shutdown();
        return false;
    }
    return true;
}


// /**
//  * @brief 领导者位姿回调函数，更新领导者的状态信息。
//  * 
//  * @param msg 来自/leader/odometry/local_filtered话题的消息指针。
//  */
// void FormationCOMM::leaderPoseCb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     ROS_INFO("leader");  
//     leader_.state.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
//     leader_.state.vel << msg->twist.twist.linear.x, msg->twist.twist.angular.z;
// }

/**
 * @brief 跟随者位姿回调函数，更新指定跟随者的状态信息。
 * 
 * @param msg 来自指定跟随者的位姿话题的消息指针。
 * @param id  机器人唯一id号。
 */
void FormationCOMM::agentsPoseCb(const nav_msgs::Odometry::ConstPtr& msg, size_t id)
{
    ++idCount_map_.at(id);  // 记录回调次数
    // ROS_INFO("机器人id: %d\n", static_cast<int>(id));

    RobotSystem& the_agent = idAgent_map_.at(id).get();
    the_agent.attributes.state.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    the_agent.attributes.state.vel << msg->twist.twist.linear.x, msg->twist.twist.angular.z;

    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    the_agent.attributes.state.heading = yaw;

    // 测试
    // ROS_INFO("id:%d, yaw:%.2f", static_cast<int>(id), the_agent.attributes.state.heading);
}


// TODO: 整理知识点
// std::unordered_set 存储不重复的元素集合
// 对于std::unordered_set::insert方法，返回的std::pair<iterator, bool>中：
// first成员是一个迭代器，它指向被插入的元素（如果插入成功）或集合中已存在的元素（如果插入失败，即元素已存在）。
// second成员是一个布尔值，指示插入操作是否成功。如果second为true，则表示元素被成功插入到集合中；
    // 如果为false，则表示元素已存在于集合中，因此没有执行插入操作。
bool FormationCOMM::checkUniqueTopics(const std::vector<RobotTopic>& topics) {
    std::unordered_set<int> seen_ids;
    std::unordered_set<std::string> seen_names;

    seen_ids.reserve(agent_num_);
    seen_names.reserve(agent_num_);

    for (const auto& topic : topics) {
        const auto& id = topic.info.id;
        const auto& name = topic.info.name;

        if (!seen_ids.insert(id).second) {
            ROS_ERROR("发现重复的机器人id: %d", id);
            return false;
        }

        if (!seen_names.insert(name).second) {
            ROS_ERROR("发现重复的机器人name: %s", name.c_str());
            return false;
        }
    }

    return true;
}


