import os


def generate_slam_launch_file(
    num_tb3,
    filename="src/slam/launch/random_movement.launch",
):

    with open(filename, "w") as f:
        f.write("<launch>\n")

        for i in range(num_tb3):
            f.write(f'    <arg name="robot{i+1}" default="tb3_{i}"/>\n')
            f.write("\n")
            f.write(
                f'    <node pkg="slam" type="random_movement.py" name="random_mover_tb3_{i}" args="$(arg robot{i+1})" output="screen"/>\n'
            )
            f.write("\n")

        f.write("</launch>\n")


if __name__ == "__main__":
    num_tb3 = int(os.getenv("NUM_OF_ROBOTS", "3"))
    generate_slam_launch_file(num_tb3)
