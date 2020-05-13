#pragma once

#include <string>
#include <unordered_map>

#include <glog/logging.h>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace VIO {

/**
* @brief The RosPublishers class
* This is a convenience class to allow for publishing msgs of the same type in
* different ROS topics.
* IdxType: typically the name of the topic (which we use to index the map of
* publishers). It must be UNIQUE, or you will override previous publishers.
* MsgType: type of the message to be sent via the publishers.
* PublisherType = ros::Publisher,  the actual ros publisher, it can be the
* classic
* ros::Publisher (default), or an image_transport::Publisher.
* NodeHandleType = ros::NodeHandle, the actual ros node handle, it can be the
* classic ros::NodeHandle (default), or a image_transport::ImageTransport.
*/
template <class IdxType,
          class MsgType,
          class PublisherType = ros::Publisher,
          class NodeHandleType = ros::NodeHandle>
class RosPublishers {
 private:
  //! Map from unique idx to ros Publisher (either a classic ros::Publisher or
  //! a image_transport::Publisher).
  using RosPubs = std::unordered_map<IdxType, PublisherType>;

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
        pubs_() {
    // TODO(Toni): add check on PublisherType and NodeHandleType, they can only
    // be either ros::Publisher + ros::NodeHandle or image_transport::Publisher
    // + image_transport::ImageTransport. Not other combinations.
  }
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
      pubs_[idx] = createPublisher(idx, &nh_private_);
    }

    // Publish msg (use .at since pub_it might be invalid).
    pubs_.at(idx).publish(msg);
  }

  /**
   * @brief getNumSubscribersForPublisher Get the number of subscribers to a
   * given
   * publisher
   * @param idx Index of the publisher in the map (unique id).
   * @return The number of subscribers to the publisher (NOTE:
   * if there is no such publisher with index `idx` in our map of publishers,
   * we will create one!
   */
  size_t getNumSubscribersForPublisher(const IdxType& idx) {
    const auto& pub_it = pubs_.find(idx);
    if (pub_it == pubs_.end()) {
      pubs_[idx] = createPublisher(idx, &nh_private_);
    }
    // Use .at(0 since pub_it might be invalid.
    return pubs_.at(idx).getNumSubscribers();
  }

 protected:
  // We need the two below because one advertise is templated, the other is not.
  PublisherType createPublisher(const IdxType& idx, ros::NodeHandle* nh) const {
    return CHECK_NOTNULL(nh)->advertise<MsgType>(
        base_topic_name_ + safeToString(idx), queue_size_, latch_msgs_);
  }
  PublisherType createPublisher(const IdxType& idx,
                                image_transport::ImageTransport* it) const {
    return CHECK_NOTNULL(it)->advertise(
        base_topic_name_ + safeToString(idx), queue_size_, latch_msgs_);
  }

 private:
  // The two below is to get around the lack of template for std::to_string
  // for an actual string :/
  static std::string to_string(const std::string& value) { return value; }
  std::string safeToString(const IdxType& idx) const {
    using namespace std;
    return to_string(idx);
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

  //! Ros handle: can be the classic ros::NodeHandle, or a
  //! image_transport::ImageTransport
  NodeHandleType nh_private_;
};

//! Define some classics:
using ImagePublishers = RosPublishers<std::string,
                                      sensor_msgs::ImagePtr,
                                      image_transport::Publisher,
                                      image_transport::ImageTransport>;

}  // namespace VIO
