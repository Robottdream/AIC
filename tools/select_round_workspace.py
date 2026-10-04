"""Select the round-omni workspace at the user's existing native entry point.

Patch only the launcher dispatch and keep a byte-exact backup. Do not sync or
overwrite the original workspace's source tree or local configuration changes.
"""
from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1]
workspace=Path('/home/polarbear/ws_aic')
target=Path('/home/polarbear/ws_aic_mecanum_20261002')
assert (target/'ros_round_omni_start.sh').is_file()
script=workspace/'ros_competition_start.sh'
marker='[车型] 圆形全向轮；转交运行工作空间：'
source=script.read_text()
if marker not in source:
    backup=workspace/'ros_competition_start.sh.before-round-20261003'
    if backup.exists(): raise RuntimeError('Backup exists, inspect before installing another dispatcher')
    shutil.copy2(script,backup)
    repo=(root/'ros_competition_start.sh').read_text()
    block=repo[repo.index('# An explicit local selection'):repo.index('SELF=')]
    anchor='WS="$(cd "$(dirname "$0")" && pwd)"\n'
    assert source.count(anchor)==1
    script.write_text(source.replace(anchor,anchor+block),newline='\n')
(workspace/'.aic_active_workspace').write_text(str(target)+'\n')
print('SELECTED',target,'at',script)
