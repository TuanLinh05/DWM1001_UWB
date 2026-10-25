"""Display sampled raw radio diagnostics without creating range samples."""
import time
import tkinter as tk
from tkinter import ttk
from burst_metadata import quality_values


class RxQualityPanel(ttk.Frame):
    COLUMNS = ('anchor', 'RESP', 'FINAL', 'FP dBm', 'RX dBm', 'RX-FP dB',
        'F1', 'F2', 'F3', 'CIR_PWR', 'RXPACC', 'STD_NOISE', 'FP_INDEX',
        'CI raw', 'CI ppm', 'Anchor FP', 'Anchor RX')

    def __init__(self, parent):
        super().__init__(parent, padding=10)
        self.latest = None
        self.received_s = 0.0
        self.summary = tk.StringVar()
        ttk.Label(self, textvariable=self.summary, wraplength=1100).pack(anchor=tk.W)
        ttk.Label(self, text=(
            'Chất lượng RX lấy mẫu tối đa 10 Hz; range vẫn theo luồng burst. '
            'F1/F2/F3/CIR_PWR/RXPACC là thanh ghi thật tại TAG khi nhận RESP. '
            'Anchor FP/RX là khi anchor nhận FINAL. Nhiệt độ/điện áp thuộc TAG. '
            'Không có mẫu được hiển thị bằng —.'), wraplength=1100).pack(anchor=tk.W, pady=10)
        self.tree = ttk.Treeview(self, columns=self.COLUMNS, show='headings', height=8)
        for name in self.COLUMNS:
            self.tree.heading(name, text=name)
            self.tree.column(name, width=86, minwidth=62, stretch=False, anchor=tk.CENTER)
        self.tree.tag_configure('stale', foreground='#888888')
        self.tree.pack(fill=tk.X)
        scroll = ttk.Scrollbar(self, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(xscrollcommand=scroll.set)
        scroll.pack(fill=tk.X)
        self.reset_session()

    def reset_session(self):
        self.latest = None
        self.tree.delete(*self.tree.get_children())
        self.summary.set('Chưa có RX_QUALITY (0x19). Cần firmware mới và bật DIAG.')

    def ingest(self, message, received_s):
        self.latest, self.received_s = message, received_s
        self.refresh(received_s)

    def refresh(self, now_s=None):
        if self.latest is None:
            return
        q = self.latest
        age_ms = max(0, ((time.monotonic() if now_s is None else now_s)-self.received_s)*1000)
        fmt = lambda v: '—' if v is None else f'{v:.2f}' if isinstance(v, float) else str(v)
        env_age = None if q.env_age_ms is None else q.env_age_ms+age_ms
        self.summary.set(f'Boot {q.boot_id:04X} · cycle {q.cycle_seq} · TAG {q.t_us} µs · '
            f'PRF {q.prf_mhz} MHz · chu kỳ lấy mẫu ≥{q.sample_period_ms} ms · '
            f'nhận cách đây {age_ms:.0f} ms'
            f'\nTAG: {fmt(q.temperature_c)} °C · {fmt(q.vbat_v)} V · '
            f'tuổi môi trường {fmt(env_age)} ms · SAR raw {q.temp_raw}/{q.vbat_raw}')
        self.tree.delete(*self.tree.get_children())
        for a in q.anchors:
            v = quality_values(a)
            values = [a.anchor_id, int(a.resp_valid), int(a.final_valid)]
            values += [v[k] for k in ('fp_dbm', 'rx_dbm', 'nlos_db', 'fp_ampl1', 'fp_ampl2',
                'fp_ampl3', 'cir_pwr', 'rxpacc', 'std_noise', 'fp_index', 'carrier_integrator',
                'ci_ppm', 'anchor_fp_dbm', 'anchor_rx_dbm')]
            self.tree.insert('', tk.END, values=tuple(fmt(x) for x in values),
                             tags=('stale',) if age_ms>500 else ())
