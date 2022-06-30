# Optimizers
The Fuse Optimizers are the top level executable structures\
The Optimizer is launched by ROS, and instantiates additional ROS nodes to service the various sensor calls and other compute tasks

A Fuse Optimizer chooses Constraints and Variables to insert or remove from the graph at run-time to regulate the amount of processing power and memory required to solve the graph.\
During startup, an Optimizer reads configuration and loads sensor_models, motion_models, and publisher plugins, then waits for callbacks from each plugin.

The plugins create Variables or Constraints, and submit Transactions to the optimizer in a callback.\
Transactions are not carried by the ROS pub-sub model; the data-types of the Constraints and Variables are not well-defined as ROS messages, so they are carried by a callback wrapper where the callback can be processed by the ROS executors without serialising or transforming the data.

The Optimizer decides when to perform an optimisation on the assembled graph, and submits the solved graph back to the plugins in another transaction.

# Optimizer Base class
This class provides plugin loader conventions, ROS parameter handling, and a few other helper functions that perform the basic administration of an optimizer.


## Batch Optimizer
The Batch Optimizer allows the graph to grow without limit, regularly solving the state.\
This is suitable for high-latency or off-line systems that need to link measurements that are separated by a large amount of time, such as SLAM loop-closures


## Fixed-Lag Optimizer

The fixed lag optimizer removes graph entries that are older than some time horizon.\
This is suitable for local navigation tasks where long histories have little effect on the confidence of a pose in local space.

The special-case where only one measurement is preserved in the sliding window, is identical to an Extended Kalman filter.

## Fixed-Size Optimizer
(not yet available)

This filter implements a sliding window similar to the Fixed-Lag optimizer, but aims to prune the graph to some size limit.


