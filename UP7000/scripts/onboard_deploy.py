"""E2.3 deployment: default read-only plan; isolated build; explicit install/rollback.

Never run an old installer. Preserve machine UUID guards and physical configs.
Use --build-only to validate a synchronized source archive without changing live
files or services. --apply installs; only --restart restarts the fixed units.
"""
import argparse
import ast
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

SOURCE_PATHS = ['UP7000/estimator', 'UP7000/ros2_ws/src/uwb_px4_logger',
                'UP7000/scripts', 'Software/New_GUI']
UNITS = ['uwb-px4-logger.service', 'uwb-eskf-recorder.service', 'uwb-recording-control.service']
BASES = ['UP7000/estimator/uwb_eskf', 'UP7000/ros2_ws/src/uwb_px4_logger']


def targets(home, root=Path('/')):
    home = PurePosixPath(str(home).replace('\\','/'))
    if not home.is_absolute():
        raise ValueError('--home must be absolute')
    def rooted(path): return root / str(path).lstrip('/')
    return dict(binary=rooted(home/'ros2_px4_v2/install/uwb_eskf/lib/uwb_eskf/uwb_eskf_node'),
                logger=rooted(home/'UP4000/ros2_ws/install/uwb_px4_logger'),
                schema=rooted('/usr/local/libexec/up7000_eskf_config.py'),
                runner=rooted(home/'drone_tools/eskf/run_onboard.py'))


def patch_runner(text):
    """Add just the E2.2 topic; leave machine-specific UUID/path guards byte-for-byte."""
    tree = ast.parse(text)
    assignment = next((n for n in tree.body if isinstance(n, ast.Assign) and
                       any(isinstance(t, ast.Name) and t.id=='TOPICS' for t in n.targets)), None)
    if assignment is None or not isinstance(assignment.value, ast.List):
        raise ValueError('Runner TOPICS is not a literal list; inspect manually')
    values=ast.literal_eval(assignment.value)
    if '/px4/attitude_frd' in values:
        return text
    if not all(isinstance(v,str) for v in values):
        raise ValueError('Unexpected runner topics')
    lines=text.splitlines(keepends=True)
    end_line=assignment.value.end_lineno-1
    # Insert immediately before the closing list bracket, without rewriting any other byte.
    col=assignment.value.end_col_offset-1
    lines[end_line]=lines[end_line][:col]+", '/px4/attitude_frd'"+lines[end_line][col:]
    patched=''.join(lines)
    ast.parse(patched)
    return patched


def inspect_archive(content):
    """Validate every member before extraction, including CSV CRLF and link traversal."""
    with tarfile.open(fileobj=io.BytesIO(content)) as archive:
        members=archive.getmembers()
        for m in members:
            p=Path(m.name)
            name=m.name.rstrip('/')
            allowed=any(name==s or name.startswith(s+'/') or (m.isdir() and s.startswith(name+'/')) for s in SOURCE_PATHS)
            if p.is_absolute() or '..' in p.parts or not allowed:
                raise ValueError('Archive path outside approved source scope: '+m.name)
            if not (m.isdir() or m.isfile()):
                raise ValueError('Source archive links/devices are not accepted: '+m.name)
            if m.isfile() and p.suffix.lower() in ('.py','.sh','.cpp','.hpp','.h','.csv','.yaml','.xml','.txt','.cmake','.md','.json','.bash','.cfg','.ini'):
                if b'\r' in archive.extractfile(m).read():
                    raise ValueError('CR detected in source archive: '+m.name)
        names={m.name for m in members if m.isfile()}
        for required in ['UP7000/scripts/reset_tag_usb.py', 'Software/New_GUI/eskf_config.py',
                         'UP7000/estimator/uwb_eskf/test/estimator_parity.cpp']:
            if required not in names:raise ValueError('Incomplete archive: '+required)


def build_commands(source, workspace, home):
    base=[str(source/p) for p in BASES]
    packages=['--packages-select','uwb_eskf','uwb_px4_logger']
    return [
        ['colcon','build','--base-paths',*base,*packages,'--executor','sequential',
         '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DUWB_ESKF_WITH_PX4_MSGS=OFF'],
        ['colcon','test','--base-paths',*base,*packages,'--return-code-on-test-failure',
         '--event-handlers','console_direct+'],
        ['colcon','test-result','--verbose']]


def ros_run(command, cwd, home, installed=False):
    # All paths/commands are positional arguments, never shell-expanded input text.
    setup=Path(cwd)/'install/setup.bash'
    script='source /opt/ros/humble/setup.bash\n'
    script+='if [ -f "$1/ros2_px4_v2/install/setup.bash" ]; then source "$1/ros2_px4_v2/install/setup.bash"; fi\n'
    script+='export PYTHONPATH="$1/drone_tools/python${PYTHONPATH:+:$PYTHONPATH}"\n'
    if installed:script+='source "$2"\n'
    script+='shift 2\nexec "$@"'
    subprocess.run(['bash','-c',script,'e23',str(home),str(setup),*command],cwd=cwd,check=True)


def privileged_replace(source, destination, mode):
    """Atomic replacement avoids modifying a mapped/running executable inode."""
    destination=Path(destination)
    if os.access(destination.parent,os.W_OK):
        fd,name=tempfile.mkstemp(prefix='.e23-',dir=destination.parent)
        try:
            with os.fdopen(fd,'wb') as f:f.write(Path(source).read_bytes())
            os.chmod(name,mode);os.replace(name,destination)
        finally:
            if os.path.exists(name):os.unlink(name)
    else:
        name=str(destination)+'.e23-staged'
        subprocess.run(['sudo','-n','install','-m',oct(mode)[2:],str(source),name],check=True)
        subprocess.run(['sudo','-n','mv','-T',name,str(destination)],check=True)


def backup_live(live, folder):
    folder.mkdir(parents=True,exist_ok=False)
    manifest=dict(schema=1,created_utc=datetime.now(timezone.utc).isoformat(),targets={})
    for key,path in live.items():
        if not path.exists():raise ValueError('Missing live target; inspect before deployment: '+str(path))
        entry=dict(path=str(path),mode=path.stat().st_mode&0o777,symlink=os.readlink(path) if path.is_symlink() else None)
        if path.is_dir():
            if path.is_symlink():raise ValueError('Logger prefix is symlinked; inspect deployment manually')
            # Dereference internal symlinks so rollback does not depend on an old build overlay.
            shutil.copytree(path,folder/key,symlinks=False)
            entry['sha256']=tree_digest(folder/key)
        else:
            shutil.copy2(path,folder/key)
            entry['sha256']=hashlib.sha256((folder/key).read_bytes()).hexdigest()
        manifest['targets'][key]=entry
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def tree_digest(root):
    digest=hashlib.sha256()
    for path in sorted(root.rglob('*')):
        if path.is_symlink():raise ValueError('Unexpected symlink in materialized backup')
        digest.update(str(path.relative_to(root)).replace('\\','/').encode('utf-8')+b'\0')
        digest.update(str(path.stat().st_mode&0o777).encode()+b'\0')
        if path.is_file():digest.update(path.read_bytes())
    return digest.hexdigest()


def replace_logger(source, destination, suffix):
    destination=Path(destination)
    if destination.is_symlink():raise ValueError('Refuse to replace a symlinked logger prefix')
    staged=destination.with_name(destination.name+'.e23-new-'+suffix)
    retired=destination.with_name(destination.name+'.e23-retired-'+suffix)
    if staged.exists() or retired.exists():raise ValueError('Deployment staging path exists')
    shutil.copytree(source,staged,symlinks=False)
    os.rename(destination,retired)
    try:os.rename(staged,destination)
    except BaseException:
        os.rename(retired,destination);raise
    return retired


def restore_backup(folder, live, restart=False):
    folder=Path(folder).resolve()
    manifest=json.loads((folder/'manifest.json').read_text())
    entries=manifest.get('targets',{})
    if manifest.get('schema')!=1 or set(entries)!=set(live):raise ValueError('Invalid rollback manifest')
    for key,path in live.items():
        if entries[key]['path']!=str(path) or not (folder/key).exists():
            raise ValueError('Rollback paths must exactly match the four known live targets')
        actual=tree_digest(folder/key) if key=='logger' else hashlib.sha256((folder/key).read_bytes()).hexdigest()
        if actual!=entries[key]['sha256']:
            raise ValueError('Backup checksum mismatch: '+key)
    suffix=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for key,path in live.items():
        if key=='logger':replace_logger(folder/key,path,'rollback-'+suffix)
        elif entries[key]['symlink'] is not None and (path.parent/entries[key]['symlink']).is_file() and \
                hashlib.sha256((path.parent/entries[key]['symlink']).read_bytes()).hexdigest()==entries[key]['sha256']:
            # Restore original leaf link; its previous target was never overwritten.
            name=path.with_name(path.name+'.e23-link-'+suffix)
            if os.access(path.parent,os.W_OK):
                name.symlink_to(entries[key]['symlink']);os.replace(name,path)
            else:
                subprocess.run(['sudo','-n','ln','-s',entries[key]['symlink'],str(name)],check=True)
                subprocess.run(['sudo','-n','mv','-T',str(name),str(path)],check=True)
        else:privileged_replace(folder/key,path,entries[key]['mode'])
    if restart:subprocess.run(['sudo','-n','systemctl','restart',*UNITS],check=True)
    return manifest


def install_live(source, workspace, live, backup):
    """Install validated overlay with existing logger configs and runner guards intact."""
    manifest=backup_live(live,backup)  # All four complete before the first replacement.
    runner=patch_runner(live['runner'].read_bytes().decode('utf-8'))
    runner_file=workspace/'runner-preserving-machine-guards.py';runner_file.write_bytes(runner.encode('utf-8'))
    logger=workspace/'install/uwb_px4_logger'
    old_config=live['logger']/'share/uwb_px4_logger/config'
    # Preserve machine-specific ports/configs. Only update the two E2.2 message
    # rate requests (these are MAVLink stream requests, not PX4 parameters).
    if old_config.is_dir():
        for path in old_config.rglob('*'):
            if path.is_file():
                target=logger/'share/uwb_px4_logger/config'/path.relative_to(old_config)
                target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    logger_config=logger/'share/uwb_px4_logger/config/logger.yaml'
    import yaml  # ROS/colcon environment; not a dependency of dry-run.
    config=yaml.safe_load(logger_config.read_text())
    px4=config['px4_logger']['ros__parameters']
    rates=px4.get('message_rates',[])
    rates=[r for r in rates if r.split(':',1)[0] not in ('HIGHRES_IMU','ATTITUDE_QUATERNION')]
    px4['message_rates']=[*rates,'HIGHRES_IMU:200','ATTITUDE_QUATERNION:50']
    logger_config.write_text(yaml.safe_dump(config,sort_keys=False))
    try:
        privileged_replace(workspace/'install/uwb_eskf/lib/uwb_eskf/uwb_eskf_node',live['binary'],manifest['targets']['binary']['mode'])
        retired=replace_logger(logger,live['logger'],backup.name)
        manifest['retired_logger']=str(retired)
        privileged_replace(source/'Software/New_GUI/eskf_config.py',live['schema'],manifest['targets']['schema']['mode'])
        privileged_replace(runner_file,live['runner'],manifest['targets']['runner']['mode'])
    except BaseException:
        restore_backup(backup,live)
        raise
    manifest['installed_from']=str(source)
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--ref',default='HEAD');parser.add_argument('--home',type=Path,default=Path.home())
    parser.add_argument('--root',type=Path,default=Path('/'),help='Dry-run fixture prefix only')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run',action='store_true');mode.add_argument('--build-only',action='store_true')
    mode.add_argument('--apply',action='store_true');mode.add_argument('--rollback',type=Path)
    parser.add_argument('--restart',action='store_true')
    parser.add_argument('--workspace',type=Path)
    parser.add_argument('--source-archive',type=Path,help='PC-created git -c core.autocrlf=false archive (sha256 required)')
    parser.add_argument('--archive-sha256')
    args=parser.parse_args(argv)
    try:
        live=targets(args.home,args.root.resolve())
        if args.restart and not (args.apply or args.rollback):raise ValueError('--restart requires --apply or --rollback')
        if args.root.resolve()!=Path('/') and (args.apply or args.build_only or args.rollback):
            raise ValueError('--root is for read-only fixture dry-run only')
        workspace=(args.workspace or args.home/('e23_deploy_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))).resolve()
        plan=dict(mode='rollback' if args.rollback else 'apply' if args.apply else 'build-only' if args.build_only else 'dry-run',
                  live_targets={k:str(v) for k,v in live.items()},workspace=str(workspace),
                  archive=['git','-c','core.autocrlf=false','archive','--format=tar',args.ref,'--',*SOURCE_PATHS],
                  source_archive=str(args.source_archive) if args.source_archive else None,
                  archive_sha256=args.archive_sha256,
                  commands=build_commands(workspace/'source',workspace,args.home),
                  test_environment='PYTHONPATH=$HOME/drone_tools/python; same --base-paths for build and test',
                  restart_units=UNITS if args.restart else [],
                  preserved='HDD UUID guards, physical rig/lever/yaw, sessions and logger ports; runner TOPICS-only patch; logger requests HIGHRES_IMU:200 and ATTITUDE_QUATERNION:50')
        print(json.dumps(plan,indent=2),flush=True)
        if not (args.apply or args.build_only or args.rollback):return 0
        if args.rollback:
            restore_backup(args.rollback,live,args.restart);return 0
        if workspace.exists():raise ValueError('Use a new --workspace; never reuse a previous build')
        if args.apply:
            for path in live.values():
                if not path.exists():raise ValueError('Missing live target: '+str(path))
            patch_runner(live['runner'].read_bytes().decode('utf-8'))  # Validate before build or install.
            subprocess.run(['sudo','-n','true'],check=True)
        if args.source_archive:
            content=args.source_archive.read_bytes()
            if not args.archive_sha256 or hashlib.sha256(content).hexdigest()!=args.archive_sha256.lower():
                raise ValueError('Source archive sha256 is required and must match')
        else:
            content=subprocess.check_output(plan['archive'],cwd=args.repo)
        inspect_archive(content)
        workspace.mkdir(parents=True,exist_ok=False)
        source=workspace/'source';source.mkdir()
        with tarfile.open(fileobj=io.BytesIO(content)) as archive:archive.extractall(source)
        (workspace/'source_archive.sha256').write_text(hashlib.sha256(content).hexdigest()+'\n')
        (workspace/'deployment_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        commands=build_commands(source,workspace,args.home)
        for i,command in enumerate(commands):ros_run(command,workspace,args.home,installed=i>0)
        # colcon can report success with zero tests: require both packages and positive counts.
        xml=list((workspace/'build').rglob('*.xml'))
        from xml.etree import ElementTree as ET
        counts={name:0 for name in ('uwb_eskf','uwb_px4_logger')}
        for path in xml:
            try:tree=ET.parse(path).getroot()
            except ET.ParseError:continue
            if tree.tag in ('testsuites','testsuite'):
                for name in counts:
                    if name in path.relative_to(workspace/'build').parts:
                        counts[name]+=sum(1 for _ in tree.iter('testcase'))
        if counts['uwb_eskf']<4 or counts['uwb_px4_logger']<1:
            raise ValueError('Missing test results for one or both packages: '+str(counts))
        if args.apply:
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            backup=args.home/'e23_backups'/('backup_'+stamp+'_'+workspace.name)
            backup.parent.mkdir(parents=True,exist_ok=True)
            install_live(source,workspace,live,backup)
            print('Backup/rollback manifest: '+str(backup/'manifest.json'),flush=True)
            if args.restart:subprocess.run(['sudo','-n','systemctl','restart',*UNITS],check=True)
        print('Complete: '+str(workspace),flush=True)
        return 0
    except (OSError,ValueError,subprocess.CalledProcessError) as exc:
        print('deploy_onboard_eskf: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
