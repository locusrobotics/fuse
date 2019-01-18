# Variables

A Variable _type_ represents some physical or logical property that is unknown or uncertain, along with any additional
properties that are required to uniquely identify Variable instances from each other. Thus, every Variable type does
two things. First, it holds the variable's _value_, acting as a one-or-more-dimensional tuple of floating-point data
that defines the physical property. Second, it defines the variable's _identity_, holding the additional information
that distinguishes one variable instance from another. Interestingly, and somewhat counter-intuitively, the _value_
of the variable tuple **does not affect** the identity of a variable instance. The purpose of fuse is to solve for
the most likely _value_ of all of the variables, and hence the variable instance _values_ will change over time as
new evidence is accumulated. But the _identity_ of a variable instance is immutable, and independent of its _value_.

A concrete example will help explain these concepts. We have two ground robots, cleverly named R1 and R2, and we want
to estimate the pose of both robots as they drive around. Thus, the physical property of interest is the 2D pose. A
2D pose consists of three floating-point quantities: `x`, `y`, and `'yaw`. Thus our Variable type must include a
3-tuple to hold the variable's _value_. Now, what about the _identity_ portion of our Variable type? Well, we have two
different robots, so we need to differentiate between the pose of R1 and the pose of R2. Thus our Variable type must
include some sort of `robot_id` field. Different instances of our variable will set the `robot_id` field to either 'R1'
or 'R2'. Additionally, the robots move around over time. So the pose of a robot is not valid for _any_ time, but
represents the pose of the robot at a _specific_ time. And so our Variable type must also include a `timestamp` field
to differentiate a pose at time `t1` for a pose at time `t2`.

Just to reiterate the important concepts:

* A variable type defines a _value_ that may be one or more floating-point quantities.
* A variable instance's _value_ is unknown and/or uncertain. It will change over time as evidence is added.
* A variable type also defines additional properties that make up its _identity_.
* A variable instance's _identity_ is immutable.
* The _identity_ is independent of the _value_.

## Variable Design Principles

The most important consideration when creating a new Variable type is deciding what data dimensions should be
included. Including too many physical properties in a Variable type will be inefficient and cumbersome when used in
places where most of the Variable's value dimensions are unused. However, including too few physical properties in
a Variable type also leads to inefficient and cumbersome usage when even the simplest of observation models involve
many variables. This is one of those "Goldilocks principle" situations.

![Goldilocks principle](http://home.netcom.com/~swansont_2/goldilocks.jpg)

Understanding how Variable types interact with the rest of the system will help in the design of "good" Variable
types.

1. The fuse stack is designed to combine observations _of the same variable instance_ from multiple sources. In
   the robotics example above, one could imagine that the robots measure their current pose using wheel odometry as
   well as with a laserscan -- two different measurements of the same pose. In order for these two measurements to
   be combined together in fuse, **they must use the same variable type**. This means that if a variable type defines
   a 15-dimensional, but a sensor only measures 3 of those dimensions, the sensor must still consume all 15 dimensions
   and define the measurement error for all 15 dimensions (which will likely mean specifying 12 dimensions worth
   of zeros). At best, this is inconvenient. It will also limit your ability to reuse sensor models from external
   sources. Unless the external source used the exact same 15-dimensional Variable type, the external sensor will
   produce measurements that are incompatible with your Variable type.
2. A measurement can involve multiple variable instances and multiple variable types. For example, a measurement could
   involve both a 2D pose **and** and 2D velocity. This allows a system with a large number of state dimensions to be
   broken up into smaller, more reusable Variable types. Consequently, it may be tempting to break down the state in
   single-dimension quantities; e.g. `x` is one variable type, `y` is a second variable type, etc. This would allow
   the most possible variable reuse, after all. However, it also means that measurements of that 15-dimensional system
   state will involve 15 different variable inputs, while _relative_ state measurements would require 30 different
   variable inputs. This makes writing the measurement error very awkward.
3. For the purposes of efficient sparse matrix operations, all of the dimensions of a variable are treated as a group.
   If there are too many dimensions in a variable type resulting in measurements errors with large numbers of zeros,
   then that sparsity information is lost. This can lead to performing dense matrix operations on larger-than-needed
   blocks, and to suboptimal solving orders. If there are too few dimensions in a variable type, then computation will
   be wasted computing the solving order over each dimension instead of each block.
4. The Variable types are analogous to message definitions in ROS. Much of the power of ROS comes from the use of
   common message types across the ROS ecosystem. While it is possible for someone to define a custom laserscan
   message that matches their use-case exactly, no other existing code would be compatible with that custom laserscan
   message. You would be unable to use the laser filters package, or visualize the laser data in RViz. Whenever
   possible, it is preferred to reused existing messages, even if they are not a perfect fit. Similarly, because the
   fuse Variables type must match exactly to be combined together, it is always better to reuse existing Variable
   types.

For the most part, reasonable variable types will be fairly obvious: 2D position, 2D velocity, 2D acceleration, etc.
The biggest debate is generally whether to include the linear and angular information into a single variable (e.g.
a 2D pose consists of a 2D position and 2D orientation), or the if they should be separate. For fuse, it was decided
to keep the linear and angular components separate. The [`fuse_variables`](fuse_variables) package provides a set
of common, reusable 2D and 3D variable types. And submissions of new variable types are always welcome.

## Variable API

Like basically everything in fuse, the Variable system is designed to be extensible. The
[`fuse_core::Variable`](fuse_core/include/fuse_core/variable.h) base class defines the minimum interface required
for all derived variable types.

* `Derived::type() -> std::string`: All derived Variables must implement a `type()` method that returns the
  fully-qualified class name. This is of the form `namespace::ClassName`. Under most circumstances, the base class
  implementation will return the correct class name. However, templated Variable classes may need to override the
  `type()` method to return the desired name.
* `Derived::size() -> size_t`: The derived Variable type must return the number of dimensions of the variable. This
  will likely just return a fixed constant.
* `Derived::data() -> double*`: The _value_ portion of the derived Variable must be accessible from a contiguous memory
  location of size `derived.size() * sizeof(double)`.
* `Derived::uuid() -> fuse_core::UUID`: This is the _identity_ of the variable. Each derived class is required to
  return a unique ID to act as the identity of the variable. The UUID must be different for each different combination
  of class and _identity_ property values. In the robotics example, the 2D pose UUID should be built from the 2D pose
  type string, the `robot_id` field, and the `timestamp` field. Importantly, if a Variable instance has the same
  _identity_ property values, **the same UUID should be returned**. This is how multiple sensors can produce the same
  variable identity in a distributed fashion. Again, in the robotics example, the UUID of a 2D pose with a `robot_id`
  of 'R1' and a `timestamp` of 10.0 produced by the odometry sensor should be the same UUID of a 2D pose with a
  `robot_id` of 'R1' and a `timestamp` of 10.0 produced by the laserscan sensor. Some functions for generating UUIDs
  are provided [here](fuse_core/include/fuse_core/uuid.h).
* `Derived::print(std::ostream& stream)`: It's nice to print variable information during debugging. Each
  derived Variable is required to implement a `print()` method, but the details of exactly what to print are left to
  the derived Variable class designer. At a minimum, the variable `uuid()` is suggested.
* `Derived::clone() -> fuse_core::Variable::UniquePtr`: All derived variables are required to implement a `clone()`
  method. This should be implemented as `return Derived::make_unique(*this)`. Because this definition requires the use
  of the derived type, a common implementation could not be provided in the base class.
* `Derived::localParameterization() -> ceres::LocalParameterization*`: This is a complex topic on its own. See the
  [Ceres documentation](http://ceres-solver.org/nnls_modeling.html#localparameterization) for an in-depth discussion
  of "local parameterizations" and their uses. If the derived Variable type requires a local parameterization, this
  method may be overridden to provide it.
* It is highly recommended that all derived Variable types include `SMART_PTR_DEFINITIONS(Derived);` in the public
  interface. This defines some common smart pointer aliases.

Additional member properties and member functions may be added to the derived Variable class. These can only be used
when an object is created with a known type. Despite this limitation, providing some syntax sugar is encouraged, as
it can make working with the derived Variable object more satisfying. For example, `fuse_variables` classes provide
named accessors for the individual dimension values. This allows use of `var.y()` in lieu of `var.data()[1]`.
