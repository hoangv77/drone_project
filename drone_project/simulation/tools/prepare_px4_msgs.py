#!/usr/bin/env python3
"""Stage px4_msgs metadata with exact local firmware schemas, without global edits."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def stage(px4, template, destination):
    if destination.exists():
        raise ValueError('Destination must be new; select a new overlay directory')
    destination.mkdir(parents=True)
    for filename in ['CMakeLists.txt','package.xml','LICENSE','README.md']:
        source=template/filename
        if source.exists(): shutil.copy2(source,destination/filename)
    if (template/'cmake').exists(): shutil.copytree(template/'cmake',destination/'cmake')
    schemas=destination/'msg'; schemas.mkdir()
    manifest={}
    # Versioned schemas take precedence over any unversioned names.
    for folder in [px4/'msg',px4/'msg/versioned']:
        for source in folder.glob('*.msg'):
            shutil.copy2(source,schemas/source.name)
            manifest[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
    # Services are not all exported by DDS, but the package metadata can refer
    # to them. Use firmware definitions rather than an unrelated main branch.
    if (px4/'srv').exists(): shutil.copytree(px4/'srv',destination/'srv')
    (destination/'firmware_schema_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return len(manifest)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--px4',type=Path,required=True)
    p.add_argument('--template',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print('Staged',stage(a.px4,a.template,a.output),'firmware message schemas')
