import pandas as pd
from testflows.combinatorics import Covering

# Step 1: Define your parameter values
param_values = {
    'update_frequency': ['1.0', '5.0', '10.0'],
    'resolution': ['0.1', '0.2'],
    'scan-obstacle_max_range': ['1.0', '2.0', '3.0'],
    'scan-raytrace_max_range': ['1.0', '2.0', '3.0'],
    'vx-z_voxels': ['5', '10', '15'],  # Will be adjusted later based on constraint
    'vx-publish_voxel_map': ['true', 'false'],
    'vx-unknown_threshold': ['5', '10', '15'],
    'vx-mark_threshold': ['0', '1', '2'],
    'inf-inflation_radius': ['0.25', '0.5', '0.75'],
    'inf-cost_scaling_factor': ['1.0', '2.0', '3.0'],
    'rolling_window': ['true', 'false'],
    'width': ['1', '2', '3'],
    'height': ['1', '2', '3'],
    'track_unknown_space': ['true', 'false'],
    'transform_tolerance': ['0.1', '0.5', '1.0'],
    'subscribe_to_updates': ['true', 'false'],
    'inflate_unknknown': ['true', 'false'],
    'inflate_around_unknown': ['true', 'false'],
    'plugin_combination': [
        "['static_layer', 'obstacle_layer', 'inflation_layer']",
        "['voxel_layer', 'static_layer', 'obstacle_layer', 'inflation_layer']",
        "['obstacle_layer', 'static_layer', 'inflation_layer']"
    ]
}

# Step 2: Generate 3-wise combinations using IPOG algorithm
covering_array = Covering(param_values, strength=3)

# Step 3: Convert to pandas DataFrame
df = pd.DataFrame(list(covering_array))

# Step 4: Apply constraints
def apply_constraints(row):
    if row['vx-publish_voxel_map'] == 'false':
        row['vx-z_voxels'] = 'IGNORED'
    return row

df = df.apply(apply_constraints, axis=1)

# Step 5: Save to CSV
csv_path = "3wise_costmap_configurations.csv"
df.to_csv(csv_path, index=False)
print(f"✅ Generated {len(df)} configurations saved to {csv_path}")
print(df.head())
