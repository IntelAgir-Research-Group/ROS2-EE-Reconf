import pandas as pd
import yaml
import ast
import os

# === CONFIG ===
CSV_FILE = "pairwise_costmap_configurations.csv"
OUTPUT_DIR = "gen_configs/local"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# === Local Costmap YAML builder ===
def build_local_costmap_yaml(row, index):
    try:
        plugins = ast.literal_eval(row['local_costmap_plugins'])
    except Exception:
        plugins = []

    # Base ROS parameters
    ros_params = {
        'update_frequency': float(row['update_frequency']),
        'publish_frequency': 2.0,
        'global_frame': 'odom',
        'robot_base_frame': 'base_link',
        'use_sim_time': True,
        'rolling_window': row['rolling_window'] == 'true',
        'resolution': float(row['resolution']),
        'transform_tolerance': float(row['transform_tolerance']),
        'subscribe_to_updates': row['subscribe_to_updates'] == 'true',
        'track_unknown_space': row['track_unknown_space'] == 'true',
        'inflate_unknknown': row['inflate_unknknown'] == 'true',
        'inflate_around_unknown': row['inflate_around_unknown'] == 'true',
        'plugins': plugins
    }

    if row['width'] != 'IGNORED':
        ros_params['width'] = int(row['width'])
    if row['height'] != 'IGNORED':
        ros_params['height'] = int(row['height'])

    # === Obstacle Layer ===
    if 'obstacle_layer' in plugins:
        ros_params['obstacle_layer'] = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan',
            'scan': {
                'data_type': 'LaserScan',
                'topic': '/scan',
                'marking': True,
                'clearing': True,
                'obstacle_max_range': float(row['scan-obstacle_max_range']) if row['scan-obstacle_max_range'] != 'IGNORED' else 2.5,
                'raytrace_max_range': float(row['scan-raytrace_max_range']) if row['scan-raytrace_max_range'] != 'IGNORED' else 3.0
            }
        }

    # === Voxel Layer (Optional, if included) ===
    if 'voxel_layer' in plugins:
        ros_params['voxel_layer'] = {
            'plugin': 'nav2_costmap_2d::VoxelLayer',
            'enabled': True,
            'z_voxels': int(row['vx-z_voxels']) if row['vx-z_voxels'] != 'IGNORED' else 10,
            'publish_voxel_map': row['vx-publish_voxel_map'] == 'true',
            'unknown_threshold': int(row['vx-unknown_threshold']) if row['vx-unknown_threshold'] != 'IGNORED' else 15,
            'mark_threshold': int(row['vx-mark_threshold']) if row['vx-mark_threshold'] != 'IGNORED' else 1
        }

    return {
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': ros_params
            }
        }
    }

# === Main execution ===
def generate_local_costmap_yamls(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    for idx, row in df.iterrows():
        config = build_local_costmap_yaml(row, idx)
        filename = os.path.join(output_dir, f"local_costmap_config_{idx+1:03d}.yaml")
        with open(filename, 'w') as file:
            yaml.dump(config, file, sort_keys=False)
    print(f"✅ {len(df)} local costmap YAML files written to '{output_dir}'")

# === Run ===
if __name__ == "__main__":
    generate_local_costmap_yamls(CSV_FILE, OUTPUT_DIR)