import os
import random


def generate_turtlebot3_launch_file(
    model,
    num_tb3,
    filename="src/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_turtlebot3.launch",
):
    random.seed(42)  # Seed for reproducibility
    positions = [
        (random.uniform(-3.0, 3.0), random.uniform(-3.0, 3.0), random.uniform(0, 3.14))
        for _ in range(num_tb3)
    ]

    with open(filename, "w") as f:
        f.write(
            """<launch>
  <arg name="model" default="{model}" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="num_tb3" default="{num_tb3}" doc="number of TurtleBot3 robots"/>

  <!-- Common parameters for Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
""".format(
                model=model, num_tb3=num_tb3
            )
        )

        for i in range(num_tb3):
            x_pos, y_pos, yaw = positions[i]
            f.write(
                """
  <group ns="tb3_{i}">
    <arg name="x_pos" default="{x_pos}"/>
    <arg name="y_pos" default="{y_pos}"/>
    <arg name="z_pos" default="0.0"/>
    <arg name="yaw" default="{yaw}"/>

    <param name="robot_description" command="$(find xacro)/xacro $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
      <param name="publish_frequency" type="double" value="50.0" />
      <param name="tf_prefix" value="tb3_{i}" />
    </node>

    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model tb3_{i} -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -Y $(arg yaw) -param robot_description" output="screen"/>
  </group>
""".format(
                    i=i, x_pos=x_pos, y_pos=y_pos, yaw=yaw
                )
            )

        f.write("</launch>")


if __name__ == "__main__":
    model = os.getenv("TURTLEBOT3_MODEL", "burger")
    num_tb3 = int(os.getenv("NUM_OF_ROBOTS", "3"))
    generate_turtlebot3_launch_file(model, num_tb3)
