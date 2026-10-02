## Named Positions

You can move the robot to a predefined joint pose from YAML or JSON using the
helper script `go_to_named_position.py`. The default example file is
[`config/named_poses.yaml`](./config/named_poses.yaml).

Example:

```bash
ros2 run annin_ar4_driver go_to_named_position.py \
	--file $(ros2 pkg prefix --share annin_ar4_driver)/config/named_poses.yaml \
	--name home
```

The file can contain multiple named poses such as `home` or `upright`. Joint
values are sent to the existing `joint_trajectory_controller` action.

If you want to execute several poses one after another from Bash, use
`run_named_positions.sh`:

```bash
ros2 run annin_ar4_driver run_named_positions.sh home upright home
```

You can also override the pose file and the pause between poses with
`--file` and `--pause`.