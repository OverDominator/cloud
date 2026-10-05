#!/usr/bin/env python3
"""Offline check: no ROS nodes, model downloads, or inference are started."""
import argparse
import importlib.util
import importlib.metadata
import pathlib
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--model-dir', type=pathlib.Path)
args = parser.parse_args()
missing = []
for module, distribution in [('rospy', None), ('cv_bridge', None),
                              ('cv2', None), ('numpy', 'numpy'),
                              ('torch', 'torch'), ('transformers', 'transformers')]:
    found = importlib.util.find_spec(module) is not None
    version = ''
    if found and distribution:
        try:
            version = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            version = 'version unavailable'
    print(('FOUND' if found else 'MISSING'), module, version)
    if not found:
        missing.append(module)
if args.model_dir:
    directory = args.model_dir.expanduser()
    for filename in ('config.json', 'preprocessor_config.json'):
        exists = (directory / filename).is_file()
        print(('FOUND' if exists else 'MISSING'), directory / filename)
        if not exists:
            missing.append(filename)
    weights = list(directory.glob('*.safetensors')) + list(directory.glob('pytorch_model*.bin'))
    print('WEIGHT_FILES', len(weights))
    if not weights:
        missing.append('model weights')
else:
    print('MODEL NOT CHECKED: supply --model-dir for a local model directory.')
print('Discovery only: import compatibility, weight integrity and inference remain untested.')
sys.exit(1 if missing else 0)
