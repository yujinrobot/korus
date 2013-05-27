== Running ==

Can use with either the sim or the real robot. Example usage:

@code
roslaunch korus simulation.launch        // in first shell
roslaunch korus send_head_pi_2.launch    // in second shell
@endcode

== Monitoring ==

Run the following to watch the motion of the joints.

@code
rostopic echo /robot_core/head_controller/state
@endcode