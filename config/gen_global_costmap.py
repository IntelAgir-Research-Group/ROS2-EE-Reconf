import pandas as pd
import yaml
import ast
import os

# === CONFIG ===
CSV_FILE = "pairwise_costmap_configurations.csv"
OUTPUT_DIR = "gen_configs/global"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# === YAML builder function ===
def build_yaml_from_row(row, index):
    try:
        plugins = ast.literal_eval(row['global_costmap_plugins'])
    except Exception:
        plugins = []

    # Create base structure
    ros_params = {
        'update_frequency': float(row['update_frequency']),
        'resolution': float(row['resolution']),
        'rolling_window': row['rolling_window'] == 'true',
        'transform_tolerance': float(row['transform_tolerance']),
        'subscribe_to_updates': row['subscribe_to_updates'] == 'true',
        'track_unknown_space': row['track_unknown_space'] == 'true',
        'inflate_unknknown': row['inflate_unknknown'] == 'true',
        'inflate_around_unknown': row['inflate_around_unknown'] == 'true',
        'plugins': plugins
    }

    # Optionally include width and height
    if row['width'] != 'IGNORED':
        ros_params['width'] = int(row['width'])
    if row['height'] != 'IGNORED':
        ros_params['height'] = int(row['height'])

    # Obstacle Layer
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

    # Inflation Layer
    if 'inflation_layer' in plugins:
        ros_params['inflation_layer'] = {
            'plugin': 'nav2_costmap_2d::InflationLayer',
            'enabled': True,
            'inflation_radius': float(row['inf-inflation_radius']) if row['inf-inflation_radius'] != 'IGNORED' else 0.55,
            'cost_scaling_factor': float(row['inf-cost_scaling_factor']) if row['inf-cost_scaling_factor'] != 'IGNORED' else 3.0,
            'inflate_unknown': ros_params['inflate_unknknown'],
            'inflate_around_unknown': ros_params['inflate_around_unknown']
        }

    # Static Layer
    if 'static_layer' in plugins:
        ros_params['static_layer'] = {
            'plugin': 'nav2_costmap_2d::StaticLayer',
            'enabled': True,
            'subscribe_to_updates': ros_params['subscribe_to_updates']
        }

    return {
        '/global_costmap/global_costmap': {
            'ros__parameters': ros_params
        }
    }

# === Main execution ===
def generate_yamls(csv_file, output_dir):
    df = pd.read_csv(csv_file)
    for idx, row in df.iterrows():
        config = build_yaml_from_row(row, idx)
        filename = os.path.join(output_dir, f"global_costmap_config_{idx+1:03d}.yaml")
        with open(filename, 'w') as file:
            yaml.dump(config, file, sort_keys=False)
    print(f"✅ {len(df)} YAML files written to {output_dir}")

# === Run ===
if __name__ == "__main__":
    generate_yamls(CSV_FILE, OUTPUT_DIR)
