import yaml
import os

plan = group.plan()

file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan.yaml')
with open(file_path, 'w') as file_save:
    yaml.dump(plan, file_save, default_flow_style=True)

with open(file_path, 'r') as file_open:
    loaded_plan = yaml.load(file_open)

group.execute(loaded_plan)