# fuse_core
This package provides the base class interfaces for the various fuse components. Concrete implementations of these
interfaces are provided in other packages.

## Variable
Within the fuse system, a "variable" is a convenient group of one or more related scalar values. These scalars are
treated as a block within the optimizer. If the variable is modeling a time-varying quatity, the an instance of the
variable represents value of the state at one specific point in time. The fuse system maintains a history of variable
instances, allowing measurements to involve previous states as well as the current state.

When defining a new variable type, there is a balance that must be struct between reusability and convenience. If you
define a complex composite state, it is unlikely that any other available components will use that same state
definition. If you make the state too granular, then more book-keeping and value lookups will be required to piece
together a useful concept from many smallar scalar components.

As an example, let's consider a 3D pose consisting of a 3D position (x, y, z) and a quaternion orientation
(qx, qy, qx, qw). We could define a single state for the 3D pose consisting of all 7 scalar components. Alternatively,
we could define two variables types, a 3 dimension position vector and a 4 dimension quaternion. Or we could even
define seven variable types, one for each dimension.

Within the `fuse_core` package, no concrete variables are actually created. We only define the common interface to which
all types must adhere. A set of common variable types are provided in the [`fuse_variables`](../fuse_variables)
package. And new variable types can be created outside the fuse stack completely. However, similar to how using common
ROS messages across nodes allow for code reuse, using common variable types will allow sensor models and motion models
to be shared across the community.
