#pragma once

#include <string>
#include <unordered_map>

#include <ros/ros.h>

namespace VIO {

/**
* @brief The RosPublishers class
* This is a convenience class to allow for publishing msgs of the same type in
* different ROS topics.
* IdxType: typically the name of the topic (which we use to index the map of
* publishers). It must be UNIQUE, or you will override previous publishers.
* MsgType: type of the message to be sent via the publishers.
*/
template <class IdxType, class MsgType>
class RosPublishers {
  typedef std::unordered_map<IdxType, ros::Publisher> RosPubs;

 public:
  /**
   * @brief RosPublishers Publishes a msg in a different ROS topic
   * defined by the LabelType. This is a convenience class
   * to allow for publishing msgs of the same type in different ROS topics.
   * @param nh_private Node handle to create the ROS publishers.
   * @param base_topic_name String to be prepended to all the ROS topics
   * advertised. It can be empty if need be.
   * @param queue_size Size of the publishers' queues (default: 1).
   * @param latch_msgs Set to true to send last msg to new subscribers (default:
   * true).
   */
  RosPublishers(const ros::NodeHandle& nh_private,
                const std::string& base_topic_name = "",
                const size_t& queue_size = 1u,
                const bool& latch_msgs = true)
      : base_topic_name_(base_topic_name),
        queue_size_(queue_size),
        latch_msgs_(latch_msgs),
        nh_private_(nh_private),
        pubs_() {}
  virtual ~RosPublishers() = default;

 public:
  // We don't pass queue_size, latch_msgs per function call bcs this is meant
  // to be called even when the publisher already exists.
  /**
   * @brief publish
   * Publishes a given ROS msg for a given semantic_label under the
   * same ROS topic.
   * If the given semantic_label has not been published before, a new ROS
   * topic
   * is advertised with the name given by topic_name_.
   * @param idx Publisher index, this will be used as well in the topic
   * name.
   * @param msg Actual msg to be published
   */
  void publish(const IdxType& idx, const MsgType& msg) {
    // Try to find the publisher.
    const auto& pub_it = pubs_.find(idx);

    // If publisher does not exist, we create a new one.
    if (pub_it == pubs_.end()) {
      pubs_[idx] = nh_private_.advertise<MsgType>(
          base_topic_name_ + std::to_string(idx), queue_size_, latch_msgs_);
    }

    // Publish msg (use .at since pub_it might be invalid).
    pubs_.at(idx).publish(msg);
  }

 private:
  //! Prepended string to the ROS topic that is to be advertised.
  std::string base_topic_name_;
  //! Latch msgs on publishing or not: set to true if you want to send last
  //! msg to new subscribers.
  bool latch_msgs_;
  //! Size of the publisher queue.
  size_t queue_size_;

  //! Map of indices to publishers, it contains all our publishers.
  RosPubs pubs_;

  //! Ros stuff
  ros::NodeHandle nh_private_;
};
}  // namespace VIO
