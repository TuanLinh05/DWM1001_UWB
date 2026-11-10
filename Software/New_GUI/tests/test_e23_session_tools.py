"""E2.3 IO/deployment edge cases; no drone access or production writes."""
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

GUI=Path(__file__).resolve().parents[1]
REPO=GUI.parents[1]
sys.path[:0]=[str(GUI),str(GUI/'tools')]
from eskf_config import aided_bias_rows
from onboard_session_io import clock_mapping, read_session, read_tlog
from onboard_session_report import flow_velocity

spec=importlib.util.spec_from_file_location('e23_deploy',REPO/'UP7000/scripts/onboard_deploy.py')
deploy=importlib.util.module_from_spec(spec);spec.loader.exec_module(deploy)


class SessionToolsTests(unittest.TestCase):
    def test_bias_import_rejects_directional_unknown_and_failed_calibration(self):
        doc=dict(ok=True,harmonics_used=0,corrections={'1':dict(anchor_id=1,bias_m=.02,cos_m=[],sin_m=[])})
        self.assertEqual(aided_bias_rows(doc,{1}),[[1,.02]])
        for bad in ({**doc,'ok':False},{**doc,'harmonics_used':1},
                    {**doc,'corrections':{'1':dict(anchor_id=1,bias_m=.02,cos_m=[.1],sin_m=[])}},
                    {**doc,'corrections':{'1':dict(anchor_id=2,bias_m=.02,cos_m=[],sin_m=[])}}):
            with self.assertRaises(ValueError):aided_bias_rows(bad,{1})
        with self.assertRaises(ValueError):aided_bias_rows(doc,{2})

    def test_integer_clock_reset_is_segmented_without_epoch_precision_loss(self):
        base=1_794_305_888_934_599_569
        us=np.array([1000000,1020000,1000,21000],np.uint64)
        offsets=np.array([base,base,base+2_000_000_000,base+2_000_000_000],np.int64)
        stamps=us.astype(np.int64)*1000+offsets
        logs=np.arange(4,dtype=np.int64)*20_000_000+base
        result=clock_mapping(stamps,logs,us,logs+123)
        self.assertEqual(result['resets'],1)
        self.assertEqual([s['offset_ns'] for s in result['segments']],[base,base+2_000_000_000])
        self.assertEqual(result['pairs'],4)
        self.assertFalse(clock_mapping([],[],[],[])['available'])

    def test_mavlink_flow_known_translation_and_gyro_cancellation(self):
        f=dict(integrated_x=.1,integrated_y=.2,integrated_xgyro=.01,integrated_ygyro=.02,
               distance=2.,integration_time_us=100000,quality=70)
        np.testing.assert_allclose(flow_velocity(f),[3.6,1.8,0.])
        self.assertIsNone(flow_velocity({**f,'distance':-1}))
        self.assertIsNone(flow_velocity({**f,'integration_time_us':0}))
        del f['integrated_ygyro'];self.assertIsNone(flow_velocity(f))

    def test_tlog_selects_matching_boot_and_handles_missing_dialect_fields(self):
        class Message:
            def __init__(self,kind,t,wall):self.kind,self.t,self._timestamp=kind,t,wall
            def get_type(self):return self.kind
            def to_dict(self):return dict(time_usec=int(self.t*1e6),quality=70)
        rows=iter([Message('HIGHRES_IMU',100,1000),Message('OPTICAL_FLOW_RAD',100,1000.1),
                   Message('HIGHRES_IMU',101,1001),Message('HIGHRES_IMU',1,1100),
                   Message('OPTICAL_FLOW_RAD',1.1,1100.1),Message('HIGHRES_IMU',2,1101)])
        class Connection:
            def recv_match(self):return next(rows,None)
            def close(self):pass
        class Mav:
            def mavlink_connection(self,*a,**k):return Connection()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'px4.tlog';path.write_bytes(b'fixture')
            with patch('onboard_session_io.dependency',return_value=Mav()):
                result=read_tlog(path,(1100_000_000_000,1102_000_000_000))
        self.assertEqual(len(result['boots']),2)
        self.assertEqual(result['selected_boot'],2)
        self.assertIsNone(result['selected']['messages']['OPTICAL_FLOW_RAD'][0]['fields'].get('signal_quality'))

    def test_mcap_multisegment_numeric_order_and_truncated_final_footer(self):
        try:
            from mcap_ros2.writer import Writer
            from mcap.writer import CompressionType
        except ImportError:
            self.skipTest('Optional offline mcap/mcap_ros2 unavailable')
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'config').mkdir();(root/'bag').mkdir()
            (root/'config/eskf.yaml').write_text(json.dumps({'uwb_eskf':{'ros__parameters':{'lever':[0,0,0]}}}))
            (root/'config/rig_onboard.txt').write_text('an1_p,0,0,0\n')
            for index,log in [(10,2000000000),(2,1000000000)]:
                path=root/f'bag/bag_{index}.mcap'
                with path.open('wb') as file:
                    with Writer(file,compression=CompressionType.NONE) as writer:
                        schema=writer.register_msgdef('std_msgs/msg/String','string data')
                        writer.write_message('/uwb_eskf/status',schema,{'data':'{"state":"WAIT_CONFIGURATION"}'},log_time=log)
                if index==10:
                    path.write_bytes(path.read_bytes()[:-20])
            data=read_session(root)
            self.assertEqual([Path(p).name for p in data.audit['files']],['bag_2.mcap','bag_10.mcap'])
            self.assertEqual(data.streams['/uwb_eskf/status']['samples'],2)
            self.assertEqual(data.audit['warnings'][0]['code'],'INCOMPLETE_FINAL_MCAP')


class DeploymentTests(unittest.TestCase):
    def test_dry_run_has_no_filesystem_writes(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'sentinel').write_text('keep')
            def snapshot():return {str(p.relative_to(root)):(p.stat().st_mtime_ns,p.read_bytes()) for p in root.rglob('*') if p.is_file()}
            before=snapshot()
            result=subprocess.run([sys.executable,'-B',str(REPO/'UP7000/scripts/onboard_deploy.py'),
                                   '--root',temp,'--home','/home/thoa','--workspace',str(root/'new')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(before,snapshot())
            self.assertFalse((root/'new').exists())
            plan=json.loads(result.stdout)
            self.assertEqual(plan['restart_units'],[])
            self.assertIn('--base-paths',plan['commands'][1])
            self.assertEqual(plan['commands'][0][3:5],plan['commands'][1][3:5])

    def test_runner_patch_keeps_machine_guards_and_is_idempotent(self):
        original="UUID = 'actual-HDD-UUID'\nMOUNT = '/mnt/custom'\nTOPICS = ['/px4/status',\n          '/uwb_eskf/odom']\nTOPICS += ['more']\n"
        patched=deploy.patch_runner(original)
        self.assertIn("UUID = 'actual-HDD-UUID'\nMOUNT = '/mnt/custom'",patched)
        self.assertEqual(patched.replace(", '/px4/attitude_frd'",''),original)
        self.assertEqual(deploy.patch_runner(patched),patched)

    def test_archive_rejects_crlf_and_path_traversal(self):
        def archive(name,data):
            output=io.BytesIO()
            with tarfile.open(fileobj=output,mode='w') as tar:
                member=tarfile.TarInfo(name);member.size=len(data);tar.addfile(member,io.BytesIO(data))
            return output.getvalue()
        for name,data in [('UP7000/estimator/test.csv',b'external\r\n'),('../outside.py',b'a\n')]:
            with self.assertRaises(ValueError):deploy.inspect_archive(archive(name,data))

    def test_backup_restore_all_four_targets_without_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);live=deploy.targets('/home/thoa',root)
            for key,path in live.items():
                path.parent.mkdir(parents=True,exist_ok=True)
                if key=='logger':path.mkdir();(path/'logger.py').write_text('old logger')
                else:path.write_text('old '+key)
            backup=root/'backup';deploy.backup_live(live,backup)
            for key,path in live.items():
                if key=='logger':(path/'logger.py').write_text('new logger')
                else:path.write_text('new '+key)
            with patch.object(deploy.subprocess,'run') as run:
                deploy.restore_backup(backup,live)
                run.assert_not_called()
            for key,path in live.items():
                self.assertEqual((path/'logger.py').read_text() if key=='logger' else path.read_text(),'old '+key)


if __name__=='__main__':unittest.main()
