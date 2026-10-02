"""Host CLI: one real Compose service with mutually exclusive autonomous/record modes."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_real'))
from hsl_real.config import load_config,load_recording


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['build','start','bag','enable','pause','stop','status','logs','shell'])
    parser.add_argument('--config',default=os.environ.get('HSL_REAL_CONFIG',str(ROOT/'config/real.yaml')))
    parser.add_argument('--no-keyboard',action='store_true',help='Leave keyboard detached; attach manually for headless tests')
    parser.add_argument('--drivers-disabled',action='store_true',help='Explicit replay/test only: do not start hardware')
    args=parser.parse_args()
    path=Path(args.config).resolve()
    cfg,mission=({'ros_domain_id':26},None)
    if args.action in ('build','start','bag','enable'):cfg,mission=load_config(path)
    env=os.environ.copy()
    env.update(HSL_REAL_CONFIG_DIR=str(path.parent),HSL_REAL_CONFIG_NAME=path.name,HSL_REAL_DOMAIN_ID=str(cfg['ros_domain_id']))
    data=Path(os.environ.get('HSL_REAL_DATA_DIR',str(ROOT/'recordings'))).resolve()
    env['HSL_REAL_DATA_DIR']=str(data)
    compose=['docker','compose','--project-name','hsl-real','--env-file',str(ROOT/'.env'),'-f',str(ROOT/'docker/docker-compose.real.yaml')]
    def run(*tail,**kw):return subprocess.run(compose+list(tail),cwd=ROOT,env=env,**kw)
    def active():
        result=run('ps','-q','real',capture_output=True,text=True,check=True)
        identifier=getattr(result,'stdout','').strip()
        if not identifier:return None
        result=subprocess.run(['docker','inspect',identifier],capture_output=True,text=True,check=True)
        info=json.loads(result.stdout)[0]
        return info['Config'].get('Labels',{})
    def control(action,manual=False):
        script=(ROOT/'tools'/('record_control.py' if manual else 'real_control.py')).read_text()
        command='source /solution/install/setup.bash && python3 - '+action
        if action=='enable' and not manual and cfg.get('localization')=='amcl':
            command+=' --require-localization'
        if args.action=='stop' and not manual:command+=' --timeout 3'
        subprocess.run(compose+['exec','-T','real','bash','-lc',command],cwd=ROOT,env=env,input=script,text=True,check=True)
    def stop(labels):
        if labels is None:
            run('stop','--timeout','90','real',check=True)
            print('No running real stack.')
            return True
        mode=(labels or {}).get('org.hsl.real.mode','autonomous')
        failed=False
        try:
            control('pause',manual=mode == 'bag')
        except (subprocess.CalledProcessError,RuntimeError) as exc:
            failed=True
            print('Motion/save service failed: '+str(exc)+'. Stopping drivers; existing files are preserved.',file=sys.stderr)
        finally:run('stop','--timeout','90','real',check=True)
        if labels and mode == 'bag':
            session=Path(labels.get('org.hsl.real.data_dir',str(data)))/labels.get('org.hsl.real.session','')
            print('Session files: '+str(session),flush=True)
            if mode=='bag' and not (session/'bag'/'metadata.yaml').is_file():
                failed=True
                print('Bag metadata.yaml missing: final export UNCONFIRMED.',file=sys.stderr)
            manifest=session/'session.json'
            if manifest.is_file():
                state=json.loads(manifest.read_text());state.update(stopped_utc=datetime.now(timezone.utc).isoformat(),final_export_confirmed=not failed)
                manifest.write_text(json.dumps(state,indent=2))
        return not failed
    if args.action=='build':
        run('build','real',check=True)
    elif args.action in ('start','bag'):
        if not args.drivers_disabled and not Path(cfg['kobuki_port']).exists():
            raise FileNotFoundError('Kobuki port not found: '+cfg['kobuki_port'])
        if args.action == 'bag' and not args.no_keyboard and not sys.stdin.isatty():
            raise RuntimeError('Keyboard requires an interactive terminal; use --no-keyboard only for explicit replay/tests')
        rec=load_recording(path.parent/'recording.yaml') if args.action == 'bag' else None
        labels=active()
        if labels and not stop(labels):raise RuntimeError('Previous session could not be saved; inspect files before restarting')
        if args.action=='start':
            env.update(HSL_REAL_MODE='autonomous',HSL_REAL_LAUNCH='robot.launch.py',HSL_REAL_LAUNCH_ARGS='drivers_enabled:='+str(not args.drivers_disabled).lower())
        else:
            session=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+args.action
            directory=data/session;directory.mkdir(parents=True)
            for file in (path,Path(cfg['mission_file']),path.parent/'recording.yaml',Path(cfg['livox_config'])):
                shutil.copy2(file,directory/file.name)
            (directory/'session.json').write_text(json.dumps({'mode':args.action,'created_utc':session,'config':str(path),'drivers_enabled':not args.drivers_disabled,'code_revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'working_tree_dirty':bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip())},indent=2))
            env.update(HSL_REAL_MODE=args.action,HSL_REAL_SESSION=session,HSL_REAL_LAUNCH='record.launch.py',
                HSL_REAL_LAUNCH_ARGS=f'session_dir:=/records/{session} drivers_enabled:={str(not args.drivers_disabled).lower()}')
            print('Recording session: '+str(directory),flush=True)
        run('up','-d','--force-recreate','real',check=True)
        if args.action=='start':print('Drivers/navigation started; motion CLOSED. Check logs_real, then enable_real.')
        elif not args.no_keyboard:
            print('Keyboard: i forward, comma backward, j/l turn, k stop. Ctrl+C closes keyboard; stop_real saves/stops session.',flush=True)
            command='source /solution/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/real/keyboard_cmd_vel '+f"-p speed:={mission['motion']['max_speed']} -p turn:={mission['motion']['max_angular_speed']} -p repeat_rate:=10.0 -p key_timeout:={rec['keyboard_timeout_s']}"
            try:run('exec','real','bash','-lc',command,check=False)
            except KeyboardInterrupt:pass
            print('Keyboard detached. Run helm stop_real to finish and save.',flush=True)
    elif args.action in ('enable','pause'):
        labels=active();manual=(labels or {}).get('org.hsl.real.mode') == 'bag'
        control(args.action,manual=manual)
    elif args.action=='stop':
        if not stop(active()):raise SystemExit(1)
    elif args.action=='status':run('ps','--all',check=True)
    elif args.action=='logs':run('logs','-f','real',check=True)
    elif args.action=='shell':run('exec','real','bash',check=True)


if __name__=='__main__':main()
