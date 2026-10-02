"""Host CLI for a separate real robot Compose project."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_real'))
from hsl_real.config import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action',choices=['build','start','enable','pause','stop','status','logs','shell'])
    parser.add_argument('--config',default=os.environ.get('HSL_REAL_CONFIG',str(ROOT/'config/real.yaml')))
    args=parser.parse_args()
    config_path=Path(args.config).resolve()
    # Stop/logs must remain usable even when an edited configuration is invalid.
    cfg = {'ros_domain_id': 26}
    if args.action in ('build','start','enable'):
        cfg,_=load_config(config_path)
    env=os.environ.copy()
    env.update(HSL_REAL_CONFIG_DIR=str(config_path.parent),HSL_REAL_CONFIG_NAME=config_path.name,
               HSL_REAL_DOMAIN_ID=str(cfg['ros_domain_id']))
    compose=['docker','compose','--project-name','hsl-real','--env-file',str(ROOT/'.env'),
             '-f',str(ROOT/'docker/docker-compose.real.yaml')]
    def run(*tail,check=True):
        return subprocess.run(compose+list(tail),cwd=ROOT,env=env,check=check)
    def control(action):
        script=(ROOT/'tools/real_control.py').read_text()
        return subprocess.run(compose+['exec','-T','real','bash','-lc',
            'source /autoware/install/setup.bash && source /drivers/install/setup.bash && python3 - '+action+
            (' --timeout 3' if args.action == 'stop' else '')],
            cwd=ROOT,env=env,input=script,text=True,check=True)
    if args.action=='build':
        subprocess.run(['helm','build','duel'],cwd=ROOT,env=env,check=True)
        run('build','real')
    elif args.action=='start':
        # A new stage requires restarting odometry and its configured start anchor.
        if not Path(cfg['kobuki_port']).exists():
            raise FileNotFoundError('Kobuki port not found: '+cfg['kobuki_port']+'; check USB/udev or real.yaml')
        run('up','-d','--force-recreate','real')
        print('Drivers and navigation started; motion is CLOSED. Check helm logs_real, then helm enable_real.')
    elif args.action in ('enable','pause'):
        control(args.action)
    elif args.action=='stop':
        try:control('pause')
        except subprocess.CalledProcessError:
            print('Permission service unavailable; stopping drivers (Kobuki command timeout 0.6s).',file=sys.stderr)
        finally:run('stop','--timeout','5','real')
    elif args.action=='status':run('ps','--all')
    elif args.action=='logs':run('logs','-f','real')
    elif args.action=='shell':run('exec','real','bash')


if __name__=='__main__':main()
