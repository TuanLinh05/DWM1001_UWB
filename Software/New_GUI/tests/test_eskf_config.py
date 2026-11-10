import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from eskf_config import normalize_config, config_hash, render_files


def configuration(confirmed=False):
    points = [(0,0,.4),(3,0,.4),(3,3,.4),(0,3,.4),(0,0,2),(3,0,2),(3,3,2),(0,3,2)]
    return {'schema':1,'confirmed':confirmed,'anchors':[[i+1,*p,True,100.,30.] for i,p in enumerate(points)],
            'lever_flu_m':[-.12,0.,.08],'yaw0_deg':10.,'parameters':{}}


class EskfConfigTests(unittest.TestCase):
    def test_normalization_is_stable_and_rendered_params_match_the_gui(self):
        value=configuration(True); normalized=normalize_config(value)
        self.assertEqual(config_hash(value),config_hash(json.loads(json.dumps(normalized))))
        files=render_files(value,'/HDD/config_versions/test')
        params=json.loads(files['eskf.yaml'])['uwb_eskf']['ros__parameters']
        self.assertEqual(params['lever'],[-.12,0.,.08])
        self.assertEqual(params['yaw0_deg'],10.)
        self.assertTrue(params['configuration_confirmed'])
        self.assertEqual(params['config_sha256'],config_hash(value))
        self.assertTrue(params['ignore_rig_bias'])
        self.assertNotIn('_bias,',files['rig_onboard.txt'])

    def test_nonfinite_duplicate_and_unknown_inputs_are_rejected(self):
        value=configuration();value['anchors'][0][1]=float('nan')
        with self.assertRaises(ValueError):normalize_config(value)
        value=configuration();value['anchors'][1][0]=1
        with self.assertRaises(ValueError):normalize_config(value)
        value=configuration();value['parameters']['arbitrary_topic']='other'
        with self.assertRaises(ValueError):normalize_config(value)

    def test_confirmed_3d_requires_enough_non_coplanar_enabled_anchors(self):
        value=configuration(True)
        for a in value['anchors']:a[3]=.4
        with self.assertRaisesRegex(ValueError,'non-coplanar'):normalize_config(value)
        value=configuration(True)
        for a in value['anchors'][2:]:a[4]=False
        with self.assertRaisesRegex(ValueError,'at least'):normalize_config(value)
        value['confirmed']=False
        self.assertFalse(normalize_config(value)['confirmed'])

    def test_unconfirmed_zero_anchor_config_remains_recordable(self):
        value=configuration()
        for a in value['anchors']:a[4]=False
        self.assertEqual(len(normalize_config(value)['anchors']),8)
        self.assertNotIn('an1_p',render_files(value,'/HDD/empty')['rig_onboard.txt'])


if __name__=='__main__':unittest.main()
