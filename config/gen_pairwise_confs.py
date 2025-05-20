import pandas as pd
from allpairspy import AllPairs

# Define parameter values
param_values = {
    'update_frequency': ['1.0', '5.0', '10.0'],
    'resolution': ['0.1', '0.2'],
    'scan-obstacle_max_range': ['1.0', '2.0', '3.0'],
    'scan-raytrace_max_range': ['1.0', '2.0', '3.0'],
    'vx-z_voxels': ['5', '10', '15'],
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

# Keys and values
keys = list(param_values.keys())
values = [param_values[k] for k in keys]

# Step 2: Define constraint logic
def apply_constraints(cfg):
    plugins = eval(cfg['plugin_combination'])

    # Obstacle layer missing → ignore scan params
    if 'obstacle_layer' not in plugins:
        cfg['scan-obstacle_max_range'] = 'IGNORED'
        cfg['scan-raytrace_max_range'] = 'IGNORED'

    # Voxel layer missing → ignore voxel params
    if 'voxel_layer' not in plugins:
        cfg['vx-z_voxels'] = 'IGNORED'
        cfg['vx-publish_voxel_map'] = 'false'
        cfg['vx-unknown_threshold'] = 'IGNORED'
        cfg['vx-mark_threshold'] = 'IGNORED'
    elif cfg['vx-publish_voxel_map'] == 'false':
        cfg['vx-z_voxels'] = 'IGNORED'

    # Inflation layer missing → ignore inflation params
    if 'inflation_layer' not in plugins:
        cfg['inf-inflation_radius'] = 'IGNORED'
        cfg['inf-cost_scaling_factor'] = 'IGNORED'

    # Rolling window false → width/height ignored
    if cfg['rolling_window'] == 'false':
        cfg['width'] = 'IGNORED'
        cfg['height'] = 'IGNORED'

    # Track unknown space false → unknown inflation false
    if cfg['track_unknown_space'] == 'false':
        cfg['inflate_unknknown'] = 'false'
        cfg['inflate_around_unknown'] = 'false'

    # If either unknown inflation enabled → track_unknown must be true
    if cfg['inflate_unknknown'] == 'true' or cfg['inflate_around_unknown'] == 'true':
        cfg['track_unknown_space'] = 'true'

    return cfg

# Step 3: Generate pairwise and apply constraints
pairwise_raw = list(AllPairs(values))
pairwise_valid = []

for row in pairwise_raw:
    cfg = dict(zip(keys, row))
    cfg = apply_constraints(cfg)
    pairwise_valid.append([cfg[k] for k in keys])

# Step 4: Create and save DataFrame
df_pairwise = pd.DataFrame(pairwise_valid, columns=keys)
df_pairwise.to_csv("pairwise_costmap_configurations.csv", index=False)
print(f"✅ Generated {len(df_pairwise)} pairwise configurations.")
print(df_pairwise.head())