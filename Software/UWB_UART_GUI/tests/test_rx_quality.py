"""Wire layout, raw-vs-compact semantics, CSV and calibration JSON regression."""
import csv
from dataclasses import replace
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import telemetry_protocol as tp
from burst_metadata import BurstMetadata, quality_values
from session_recorder import SessionRecorder


def quality_payload(count=8, flags=3):
    header=struct.pack(tp.QUALITY_HEADER_FORMAT,1,0xBEEF,100,2**40+5000,16,count,
                       flags,3440,3300,40,100,138,160)
    return header+b''.join(struct.pack(tp.QUALITY_ENTRY_FORMAT,i+1,3,63,101,202,303,
        404,35,0x2A40,32051,-8212,-7650,-8312,-7713) for i in range(count))


def decode(payload=None):
    data=tp.encode_frame(tp.TYPE_RX_QUALITY,100,1000,
                         quality_payload() if payload is None else payload)
    return tp.decode_frame(tp.TelemetryStreamParser().feed(data)[0])


def burst():
    a=tp.BurstAnchor(1,0x20,tp.BURST_FLAG_RADIO_OK,1234,-8200,-7640,-1825)
    return tp.RangeBurstMessage(101,1010,0xBEEF,101,800,2**40+15000,3587,(a,))


class QualityTests(unittest.TestCase):
    def test_full8_layout_and_fragmented_stream(self):
        payload=quality_payload()
        self.assertEqual((tp.QUALITY_HEADER_SIZE,tp.QUALITY_ENTRY_SIZE,len(payload)),(30,28,254))
        frame=tp.encode_frame(tp.TYPE_RX_QUALITY,100,1000,payload)
        self.assertEqual(len(frame),270)
        p=tp.TelemetryStreamParser(); frames=[]
        for byte in frame: frames+=p.feed(bytes((byte,)))
        q=tp.decode_frame(frames[0])
        self.assertEqual((q.t_us,q.temperature_c,q.vbat_v,q.env_age_ms),
                         (2**40+5000,34.4,3.3,40))
        a=q.anchors[0]
        self.assertEqual((a.fp_ampl1,a.fp_ampl2,a.fp_ampl3,a.cir_pwr,a.rxpacc),(101,202,303,404,63))
        self.assertEqual((a.fp_cdbm,a.rx_cdbm,a.carrier_integrator),(-8212,-7650,32051))
        self.assertAlmostEqual(quality_values(a)['ci_ppm'],-18.36906912)

    def test_invalid_layout_schema_count_id_and_prf(self):
        base=bytearray(quality_payload())
        cases=[base[:-1],base+b'\0']
        for offset,value in [(0,2),(15,32),(16,9),(17,4),(30,9),(31,4),(58,1)]:
            b=bytearray(base)
            b[offset]=value
            cases.append(b)
        for payload in cases:
            with self.subTest(payload=bytes(payload[:32])),self.assertRaises(tp.ProtocolError):
                decode(payload)

    def test_unknowns_and_valid_zero_registers(self):
        q=decode(quality_payload(flags=0))
        self.assertIsNone(q.temperature_c);self.assertIsNone(q.vbat_v)
        a=replace(q.anchors[0],flags=0)
        self.assertIsNone(quality_values(a)['fp_ampl1'])
        self.assertIsNone(quality_values(a)['ci_ppm'])
        self.assertEqual(quality_values(replace(a,flags=1,fp_ampl1=0))['fp_ampl1'],0)

    def test_json_works_with_old_firmware_and_caches_explicit_sample_age(self):
        builder=BurstMetadata(); b=burst()
        old=builder.build(b); a=old['anchors'][0]
        self.assertEqual((a['fp_dbm'],a['rx_dbm'],a['nlos_db'],a['ci_ppm'],a['ci']),
                         (-82.0,-76.4,5.6,-18.25,-18250))
        self.assertEqual(old['meas_seq'],800);self.assertIsNone(a['quality'])
        builder.accept(decode()); new=builder.build(b)
        q=new['anchors'][0]['quality']
        self.assertEqual((q['cycle_seq'],q['tag_time_us'],q['age_ms']),(100,2**40+5000,10))
        self.assertEqual(new['temperature_c'],34.4)
        self.assertEqual(new['env_age_ms'],50)
        self.assertEqual(new['anchors'][0]['fp_dbm'],-82.0) # current compact, not stale precise
        json.dumps(new,allow_nan=False)
        for changed in (replace(b,boot_id=2),replace(b,t_us=2**40),
                        replace(b,t_us=b.t_us+600000)):
            self.assertIsNone(builder.build(changed)['anchors'][0]['quality'])

    def test_old_vector_has_delta_then_signed_ci_and_full_u64_time(self):
        payload=bytes.fromhex('01929d4ae3030062091a00bec8f96000000000202708'
            '012020f400a838f0 0220201801a628fe 0320209901a737f1 0420209a02aa3901'
            '052020ac04a834c4 0620200c04a939ae 0720200504a82ab0 082020ea03a93aaa')
        data=tp.encode_frame(tp.TYPE_RANGE_BURST,1,0,payload)
        b=tp.decode_frame(tp.TelemetryStreamParser().feed(data)[0]); a=b.anchors[0]
        self.assertEqual((a.fp_cdbm,a.rx_cdbm,a.ci_ppm_x100),(-8400,-7840,-400))
        self.assertEqual((b.t_us,b.period_us),(1626982590,10016))

    def test_recorder_writes_one_row_per_anchor_and_keeps_sample_time(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent,
                                          prefix='.quality-test-') as temp:
            rec=SessionRecorder(Path(temp),'TEST_QUALITY',460800)
            rec.record_message(decode())
            rec.stop()
            with (rec.session_directory/'rx_quality.csv').open(encoding='utf-8',newline='') as f:
                rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),8)
            self.assertEqual((rows[0]['fp_ampl1'],rows[0]['t_us']),('101',str(2**40+5000)))
            self.assertEqual(rows[0]['temperature_c'],'34.4')
            self.assertEqual(rec.meas_records,0) # diagnostics never fabricate range measurements

    def test_quality_widget_values_staleness_and_reset(self):
        import tkinter as tk
        from rx_quality_panel import RxQualityPanel
        try:
            root=tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        root.withdraw()
        try:
            panel=RxQualityPanel(root);panel.ingest(decode(),1000.0)
            row=panel.tree.item(panel.tree.get_children()[0],'values')
            self.assertEqual((row[3],row[6],row[10]),('-82.12','101','63'))
            panel.refresh(1001.0)
            self.assertEqual(panel.tree.item(panel.tree.get_children()[0],'tags'),('stale',))
            self.assertIn('34.40',panel.summary.get())
            panel.reset_session();self.assertFalse(panel.tree.get_children())
        finally:
            root.destroy()


if __name__=='__main__':
    unittest.main()
