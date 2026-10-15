from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uwb_command import BURST_TIMING_PRESETS
from telemetry_protocol import BurstSettings


class BurstPresetTests(unittest.TestCase):
    def test_highrate_alias_and_encoding(self):
        self.assertEqual(BURST_TIMING_PRESETS["hz200"], BURST_TIMING_PRESETS["hz220"])
        preset = BURST_TIMING_PRESETS["hz220"]
        self.assertEqual(preset["period_us"], 4500)
        self.assertGreater(1_000_000 / preset["period_us"], 220)
        # The CLI settings match the high-rate firmware boot profile.
        config = Path(__file__).resolve().parents[3] / "Firmware/nodes/tag/Tag_DevKit/include/uwb_app_config.h"
        text = config.read_text(encoding="utf-8")
        for macro, field in (("BASE_UUS", "base_uus"), ("SLOT_UUS", "slot_uus"),
                             ("FINAL_MARGIN_UUS", "final_margin_uus"),
                             ("GAP_US", "gap_us"), ("PERIOD_US", "period_us")):
            self.assertRegex(text, rf"#define\s+UWB_BURST_{macro}\s+{preset[field]}U")
        settings = BurstSettings(mode=1, **preset)
        self.assertEqual(BurstSettings.decode(settings.encode()), settings)

    def test_100hz_has_final_margin_used_in_hardware_log(self):
        preset = BURST_TIMING_PRESETS["hz100"]
        self.assertEqual(preset["period_us"], 10000)
        self.assertEqual(preset["final_margin_uus"], 1200)


if __name__ == "__main__":
    unittest.main()
