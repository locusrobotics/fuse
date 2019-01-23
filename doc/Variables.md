# Variables

In the fuse stack, a Variable does two things:

## Variable Value

First, a Variable has a _value_. A Variable holds one or more dimensions of floating point data that represents some
physical or logical property. For example, a two dimensional pose consists of `x`, `y`, and `yaw` floating point
values. A single Variable class could be implemented hold all three pose dimensions. The exact value of each dimension
will be unknown and uncertain. It is the job of an Optimizer to find the most likely value of each dimension held by
each Variable instance in the system.

## Variable Identity

Second, a Variable must have an _identity_, some way of distinguishing one occurrence of a Variable from another within
the system. There are several reasons multiple identities of a Variable will be contained within a single system:

* The Variable may represent a property common to multiple entity types. Both a robot and a visual landmark have a
  position in space. Different identities of the same Variable type may be used to describe these different entities.
* Similarly, there may be multiple occurrences of the same entity type within the system. A multi-robot configuration
  will want to track the pose of each robot, and thus different identities of a Variable will be used for each robot
  in the system.
* Most commonly, a Variable will represent a time-varying process. A different Variable identity will be required
  for each time instant for which the process value is to be estimated. For example, the pose of the robot will change
  over time, so we need a unique Variable identity representing the robot pose at time `t1` **and** another unique
  Variable identity representing the robot pose at time `t2`. Any time-varying process must be discretized within
  the fuse stack.

The Variable identity takes the form of a UUID or hash value, and is generally derived from a set of additional
properties that describe what makes each occurrence unique from other occurrences. In the case of a time-varying
process, this will likely involve the timestamp. In the case of a Variable that can describe multiple robots, this
might involve the serial number of the robot.

An important aspect of the Variable identity is that the same UUID must be generated when the same Variable
identity is referenced from different places in the distributed fuse stack. For example, a robot may estimate its pose
at a specific time using wheel odometry measurements, and it may estimate its pose at the same time instant using some
laserscan matching algorithm. Both measurements involve the same Variable identity: the pose of robot `R` measured at
time `t`, and thus both sensors must generate a Variable instance with the same identity.

## Design Principles

The most important consideration when creating a new Variable type is deciding what data dimensions should be
included. Including too many physical properties in a Variable type will be inefficient and cumbersome when used in
places where most of the dimensions are unused. However, including too few physical properties in a Variable type
also leads to inefficient and cumbersome usage when even the simplest of observation models involve many variables.
This is one of those "Goldilocks principle" situations.

![Goldilocks principle](http://home.netcom.com/~swansont_2/goldilocks.jpg)

Understanding how Variable types interact with the rest of the system will help in the design of "good" Variable
types.

* The fuse stack is designed to combine observations _of the same variable identity_ from multiple sources. As
  described above, if a robot measures its current pose using wheel odometry as well as with a laserscan, then we have
  two different measurements of the same pose. In order for these two measurements to be combined together in fuse,
  **they must use the same variable type**. In a way, the Variable types are analogous to message definitions in ROS.
  Much of the power of ROS comes from the use of common message types across the ROS ecosystem. While it is possible
  for someone to define a custom laserscan message that matches their use-case exactly, no other existing code would
  be compatible with that custom laserscan message. You would be unable to use the laser filters package, or visualize
  the laser data in RViz. Whenever possible, it is preferred to reused existing messages, even if they are not a
  perfect fit. Similarly, because the fuse Variable types must match exactly to be combined together, it is always
  better to reuse existing Variable types.
* A measurement can involve multiple variable instances and multiple variable types. For example, a measurement could
  involve both a 2D pose **and** a 2D velocity. This allows a system with a large number of measured dimensions to be
  broken up into smaller, more reusable Variable types. It may be tempting to break down the state in single-dimension
  quantities; e.g. `x` is one variable type, `y` is a second variable type, etc. This would allow the most possible
  variable reuse, after all. However, it also means that certain types of measurements will involve an annoyingly
  large number of variables. For example, a 3D inertial strapdown sensor estimates the change in the 3D pose,
  3D velocity, and 3D linear acceleration. This involves a total of 15 dimensions of data measured at two different
  timestamps. If every dimension is its variable type, such a measurement would require 30 variable inputs, making
  it very awkward to write.
* Conversely, a Variable could be defined that contains all the measured dimensions of your system. In the 3D inertial
  strapdown case, this would a 15-dimensional vector representing the 3D pose, 3D velocity, and 3D linear acceleration
  at a specific timestamp. However, this means that every sensor must consume all 15 dimensions and define the error
  function in terms of all 15 dimensions, even if the sensor only measures a small subset of those dimensions. At best,
  this is inconvenient. It will also limit your ability to reuse sensor models from external sources, unless the
  external source happened to use that exact same 15-dimensional Variable type.
* For the purposes of efficient sparse matrix operations, all of the dimensions of a variable are treated as a group.
  If there are too many dimensions in a variable type, resulting in measurements errors with large numbers of zeros,
  then that sparsity information is lost. This can lead to performing dense matrix operations on larger-than-needed
  blocks, and to suboptimal solving orders. If there are too few dimensions in a variable type, then computation will
  be wasted computing the solving order over each dimension instead of each block.

For the most part, reasonable variable types will be fairly obvious: 2D position, 2D velocity, 2D acceleration, etc.
The biggest debate is generally whether to include the linear and angular information into a single variable (e.g.
a 2D pose consists of a 2D position and 2D orientation), or the if they should be separate. For fuse, it was decided
to keep the linear and angular components separate. The [`fuse_variables`](fuse_variables) package provides a set
of common, reusable 2D and 3D variable types. And submissions of new variable types are always welcome.

## Variable API

Like basically everything in fuse, the Variable system is designed to be extensible. The
[`fuse_core::Variable`](fuse_core/include/fuse_core/variable.h) base class defines the minimum interface required
for all derived variable types.

* `Derived::type() -> std::string`

  All derived Variables must implement a `type()` method that returns the fully-qualified class name. This is of the
  form `namespace::ClassName`. Under most circumstances, the base class implementation will return the correct class
  name. However, templated Variable classes may need to override the `type()` method to return the desired name.

* `Derived::size() -> size_t`

  The derived Variable type must return the number of dimensions of the variable. This will likely just return a fixed
  constant.

* `Derived::data() -> double*`

  The _value_ portion of the derived Variable must be accessible from a contiguous memory location of size
  `derived.size() * sizeof(double)`.

* `Derived::uuid() -> fuse_core::UUID`

  Each derived class is required to return a unique ID to act as the identity of the variable. Some functions for
  generating UUIDs are provided [here](fuse_core/include/fuse_core/uuid.h).

* `Derived::print(std::ostream& stream)`

  It's nice to print variable information during debugging. Each
  derived Variable is required to implement a `print()` method, but the details of exactly what to print are left to
  the derived Variable class designer. At a minimum, the variable `uuid()` is suggested.

* `Derived::clone() -> fuse_core::Variable::UniquePtr`

  All derived variables are required to implement a `clone()`
  method. This should be implemented as `return Derived::make_unique(*this)`. Because this definition requires the use
  of the derived type, a common implementation could not be provided in the base class.

* `Derived::localParameterization() -> ceres::LocalParameterization*`

  This is a complex topic on its own. See the
  [Ceres documentation](http://ceres-solver.org/nnls_modeling.html#localparameterization) for an in-depth discussion
  of "local parameterizations" and their uses. If the derived Variable type requires a local parameterization, this
  method may be overridden to provide it.

* `SMART_PTR_DEFINITIONS(Derived);`

  It is highly recommended that all derived Variable types include `SMART_PTR_DEFINITIONS(Derived);` in the public
  interface. This defines some common smart pointer aliases, such as `Derived::SharedPtr` and `Derived::UniquePtr`.

Additional member properties and member functions may be added to the derived Variable class. These can only be used
when an object is created with a known type. Despite this limitation, providing some syntax sugar is encouraged, as
it can make working with the derived Variable objects more satisfying. For example, `fuse_variables` classes provide
named accessors for the individual dimension values. This allows use of `var.y()` in lieu of `var.data()[1]`.
