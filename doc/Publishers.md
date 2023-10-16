# Publishers

The purpose of a Publisher is to publish data from the Optimizer whenever an optimization cycle completes. Publishers get access to the optimized values of all state variables, so they can output data from `fuse` into a wider system. The exact timing, frequency, and type of message published are dependent on the Publisher type used.

Most practical publishers are specifically derivatives of the `AsyncPublisher` class, which allows for any Publisher derived from it to publish on a different thread from the Optimizer. An `AsyncPublisher` maintains its own threads and local callback queue. Entries are added to the local callback queue as the Optimizer generates them, but they are serviced in a seperate thread pool from everything else. This is almost always desirable from a performance standpoint, although it does require users to ensure that any callbacks the Publisher calls are thread-safe.

## Publisher API

Like basically everything else in `fuse`, the Publisher system is designed to be extensible. The [`fuse_core::Publisher`](../fuse_core/include/fuse_core/publisher.h) defines the minimum interfaces required for all derived Publishers. 

* `Derived::initialize(std::string name)`

An initialization method. As all Publishers are nodelets, most setup should be performed in this method, instead of in a constructor. 

* `Derived::name() -> std::string`

Gets the name of the derived Publisher.

* `Derived::notify(Transaction::ConstSharedPtr transaction, Graph::ConstSharedPtr graph)`

This is the function used by the Optimizer to notify the Publisher that Graph values have been updated. The `notify` method is called in the optimizer's thread after each optimization cycle has been completed. The `transaction` parameter should include the set of added and removed variables and constraints since the last optimization cycle. This should prevent the Publisher from having to search the entire Graph in most cases. For those cases where `transaction` is insufficient, the Graph is also provided.

### `AsyncPublisher` API

The `AsyncPublisher` class is a derivate of `fuse_core::Publisher`, so it implements all of the above noted functions. However, as most practical publishers will be derived from `AsyncPublisher`, we will also define the interface for a derived `AsyncPublisher` class. 

* `AsyncDerived::onInit()`

The `AsyncPublisher` class defines its own `initialize` function, which handles setup for the internal asynchronous spinner and callback queue members. Any other initialization that needs to occur for a particular Publisher should be placed inside of this function, which will be called after initialization of the `AsyncPublisher` is complete.

* `AsyncDerived::notifyCallback(Transaction::ConstSharedPtr transaction, Graph::ConstSharedPtr graph)`

The `notify` function is overridded by the `AsyncPublisher` implementation to insert `notifyCallback` calls into a local callback queue and execute them using a local set of threads, as opposed to allowing them to be executed in the Optimizer threads. The `notifyCallback` function is where any publishing behavior should be contained.


## Example 

As a concrete example, we will review the `Path2DPublisher` class provided in [`fuse_publishers`](../fuse_publishers/).

```C++
class Path2DPublisher : public fuse_core::AsyncPublisher
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Path2DPublisher);

  Path2DPublisher::Path2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  frame_id_("map")
  {
  }

  virtual ~Path2DPublisher() = default;

  void Path2DPublisher::onInit()
  {
    // Configure the publisher
    std::string device_str;
    if (private_node_handle_.getParam("device_id", device_str))
    {
      device_id_ = fuse_core::uuid::from_string(device_str);
    }
    else if (private_node_handle_.getParam("device_name", device_str))
    {
      device_id_ = fuse_core::uuid::generate(device_str);
    }
    private_node_handle_.getParam("frame_id", frame_id_);

    // Advertise the topic
    path_publisher_ = private_node_handle_.advertise<nav_msgs::Path>("path", 1);
    pose_array_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseArray>("pose_array", 1);
  }

  void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override
    {
    // Exit early if no one is listening
    if ((path_publisher_.getNumSubscribers() == 0) && (pose_array_publisher_.getNumSubscribers() == 0))
    {
      return;
    }
    // Extract all of the 2D pose variables to the path
    std::vector<geometry_msgs::PoseStamped> poses;
    for (const auto& variable : graph->getVariables())
    {
      auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
      if (orientation &&
        (orientation->deviceId() == device_id_))
      {
        const auto& stamp = orientation->stamp();
        auto position_uuid = fuse_variables::Position2DStamped(stamp, device_id_).uuid();
        if (!graph->variableExists(position_uuid))
        {
          continue;
        }
        auto position = dynamic_cast<const fuse_variables::Position2DStamped*>(&graph->getVariable(position_uuid));
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = position->x();
        pose.pose.position.y = position->y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation->getYaw()));
        poses.push_back(std::move(pose));
      }
    }
    // Exit if there are no poses
    if (poses.empty())
    {
      return;
    }
    // Sort the poses by timestamp
    auto compare_stamps = [](const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
    {
      return pose1.header.stamp < pose2.header.stamp;
    };
    std::sort(poses.begin(), poses.end(), compare_stamps);
    // Define the header for the aggregate message
    std_msgs::Header header;
    header.stamp = poses.back().header.stamp;
    header.frame_id = frame_id_;
    // Convert the sorted poses into a Path msg
    if (path_publisher_.getNumSubscribers() > 0)
    {
      nav_msgs::Path path_msg;
      path_msg.header = header;
      path_msg.poses = poses;
      path_publisher_.publish(path_msg);
    }
    // Convert the sorted poses into a PoseArray msg
    if (pose_array_publisher_.getNumSubscribers() > 0)
    {
      geometry_msgs::PoseArray pose_array_msg;
      pose_array_msg.header = header;
      std::transform(poses.begin(),
                    poses.end(),
                    std::back_inserter(pose_array_msg.poses),
                    [](const geometry_msgs::PoseStamped& pose)
                    {
                      return pose.pose;
                    });  // NOLINT(whitespace/braces)
      pose_array_publisher_.publish(pose_array_msg);
    }
  }

protected:
  fuse_core::UUID device_id_;  //!< The UUID of the device to be published
  std::string frame_id_;  //!< The name of the frame for this path
  ros::Publisher path_publisher_;  //!< The publisher that sends the entire robot trajectory as a path
  ros::Publisher pose_array_publisher_;  //!< The publisher that sends the entire robot trajectory as a pose array
};
```

Now, let's examine this step by step.

All new variables must derive from the base Publisher class, as they are plugins. All the Publishers included in `fuse` by default derive from `fuse_core::AsyncPublisher`, and use the asynchronous callback queue provided by that class. 
```C++
class Path2DPublisher : public fuse_core::AsyncPublisher
```

As with other types of `fuse` objects, we typically use a macro to define useful smart pointer aliases.
```C++
FUSE_SMART_PTR_DEFINITIONS(Path2DPublisher);
```

As is convention for plugins, we do minimal setup in the actual constructor, defining only placeholder values here. As this class derives from `fuse_core::AsyncPublisher`, any further setup should occur in the `onInit()` function.
```C++
  Path2DPublisher::Path2DPublisher() :
  fuse_core::AsyncPublisher(1),
  device_id_(fuse_core::uuid::NIL),
  frame_id_("map")
  {
  }
```

As mentioned above, most of our setup occurs in the `onInit()` function. Here we generate our `fuse_core::UUID`, check for the frame our output data should be in, and advertise our publishers for the output data. A good rule of thumb is that each Publisher should only publish one concept, but it can publish that concept in as many different data formats as necessary. For instance, this Publisher publishes an optimized path that's output from the Optimizer as a concept, but it provides that information in both `nav_msgs::Path` and `geometry_msgs::PoseArray` formats. 
```C++
void Path2DPublisher::onInit()
{
  std::string device_str;
  if (private_node_handle_.getParam("device_id", device_str))
  {
    device_id_ = fuse_core::uuid::from_string(device_str);
  }
  else if (private_node_handle_.getParam("device_name", device_str))
  {
    device_id_ = fuse_core::uuid::generate(device_str);
  }
  private_node_handle_.getParam("frame_id", frame_id_);

  path_publisher_ = private_node_handle_.advertise<nav_msgs::Path>("path", 1);
  pose_array_publisher_ = private_node_handle_.advertise<geometry_msgs::PoseArray>("pose_array", 1);
}
```

For members of this class, we define a UUID to identify unique device information in the Graph, as we need to do in the `notifyCallback` function. We have 2 publishers to publish different formats of the path data, and a frame id to define which frame the output data should be in. 
```C++
protected:
  fuse_core::UUID device_id_;  
  std::string frame_id_;
  ros::Publisher path_publisher_; 
  ros::Publisher pose_array_publisher_;
```

Now we get into the beefy part of any Publisher, the notification callback for Optimizer results. To start off, we don't bother generating a path if there's no one subscribing to any of our publishers.
```C++
void notifyCallback(
    fuse_core::Transaction::ConstSharedPtr transaction,
    fuse_core::Graph::ConstSharedPtr graph) override
    {
      if ((path_publisher_.getNumSubscribers() == 0) && (pose_array_publisher_.getNumSubscribers() == 0))
      {
        return;
      }

```

For the `Path2DPublisher`, we're interested in the entire path of a given robot. The code below pulls that path out of the Graph. It's not guaranteed that all poses are always optimized, so the entire path will not necessarily be contained in the `transaction` provided. Instead, we iterate over the provided `graph`, making sure to check that the device IDs associated with each variable is the device ID we're interested in. 
```C++
  std::vector<geometry_msgs::PoseStamped> poses;
  for (const auto& variable : graph->getVariables())
  {
    auto orientation = dynamic_cast<const fuse_variables::Orientation2DStamped*>(&variable);
    if (orientation &&
       (orientation->deviceId() == device_id_))
    {
      const auto& stamp = orientation->stamp();
      auto position_uuid = fuse_variables::Position2DStamped(stamp, device_id_).uuid();
      if (!graph->variableExists(position_uuid))
      {
        continue;
      }
      auto position = dynamic_cast<const fuse_variables::Position2DStamped*>(&graph->getVariable(position_uuid));
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = position->x();
      pose.pose.position.y = position->y();
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), orientation->getYaw()));
      poses.push_back(std::move(pose));
    }
  }
```

We're not guaranteed to have any variables for a given UUID in a Graph, nor are we guaranteed to have variables provided in time order. We need to cover those issues. 
```C++
  if (poses.empty())
  {
    return;
  }
  
  auto compare_stamps = [](const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
  {
    return pose1.header.stamp < pose2.header.stamp;
  };
  std::sort(poses.begin(), poses.end(), compare_stamps);
```

Ensure that the header is correctly populated. Improperly formed headers can cause weird problems.
```C++
  std_msgs::Header header;
  header.stamp = poses.back().header.stamp;
  header.frame_id = frame_id_;
```

Last but not least, we do all the data conversions required for any ROS message types we're interested in using, and publish out our data.
```C++
  if (path_publisher_.getNumSubscribers() > 0)
  {
    nav_msgs::Path path_msg;
    path_msg.header = header;
    path_msg.poses = poses;
    path_publisher_.publish(path_msg);
  }
  if (pose_array_publisher_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header = header;
    std::transform(poses.begin(),
                   poses.end(),
                   std::back_inserter(pose_array_msg.poses),
                   [](const geometry_msgs::PoseStamped& pose)
                   {
                     return pose.pose;
                   });  // NOLINT(whitespace/braces)
    pose_array_publisher_.publish(pose_array_msg);
  }
```