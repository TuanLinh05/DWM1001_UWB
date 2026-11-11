# G0 — Hiệu chuẩn đa điểm, chẩn đoán mất gói radio, bộ dữ liệu chuẩn

**Ngày:** 08/11/2026
**Người thực hiện code:** Codex. **Người kiểm tra (review):** Claude. **Người chạy phần cứng:** chủ dự án.
**Phạm vi code:** `Software/New_GUI/` (Python 3, PyQt6, numpy). Không đụng firmware trừ mục G0.4 (tùy chọn).
**Nguồn gốc:** giai đoạn G0 trong `D:\Drone Project\Drone with UWB\Plan\NGHIEN_CUU_UOC_LUONG_VA_DIEU_KHIEN_NANG_CAO_UWB_MTF01_UP7000.md`. File này tự đủ thông tin; Codex không cần đọc file kia.

---

## 0. Tóm tắt cho người đọc nhanh

G0 làm sạch dữ liệu đầu vào trước khi xây bộ lọc ESKF (G2), gồm ba việc:

| Mục | Việc | Vì sao |
|---|---|---|
| G0.1 | **Hiệu chuẩn đa điểm**: giải đồng thời bias 8 anchor và vị trí thật của từng điểm đặt TAG | Hiệu chuẩn một điểm không tách được "đặt TAG sai chỗ" khỏi "bias". Log 07/11 có bias giả ±0,5 m do một điểm nhập lệch 0,65 m |
| G0.2 | **Công cụ chẩn đoán mất gói radio** theo thời gian, theo anchor, theo nhiệt độ TAG | Có phiên chỉ 1–3,5/8 anchor trả lời mỗi chu kỳ. Cần biết lỗi ở một anchor, ở TAG hay ở môi trường |
| G0.3 | **Bộ dữ liệu chuẩn**: quy trình ghi + nhãn waypoint + công cụ thống kê và nghiệm thu | Có số liệu nhiễu σ, thời gian tương quan τ, phần dư sau hiệu chuẩn tại điểm kiểm chứng độc lập |
| G0.4 | (Tùy chọn) Firmware gửi thêm `LDE_PPINDX` trong gói `0x19` | Để tính chỉ số NLOS của Decawave APS006 Part 3 ở G2 |

**Hợp đồng bàn giao:** đã có sẵn **39 test đang đỏ** cùng khung code. Codex làm cho chúng xanh **mà không sửa test**. 78 test cũ phải tiếp tục xanh.

---

## 1. Hiện trạng và khung code đã có

### 1.1 Mã hiện có liên quan

| File | Nội dung liên quan |
|---|---|
| `Software/New_GUI/core_state.py` | `UwbStateEngine`: `start_spot_calibration`, `finish_calibration` (tạo `CalibrationSpotCapture` với `CalibrationAnchorResult` cho từng anchor), `aggregated_calibration` (hiện lấy **trung vị bias từng điểm**), `calibration_warnings`, `apply_calibration_to_host`, `check_spot_consistency`, `mark_waypoint(label)` |
| `Software/New_GUI/main_gui.py` | Trang hiệu chuẩn `_build_calibration_page` (~dòng 790), `_update_calibration_page` (~1659), export CSV/JSON `_export_calibration_records_dialog` (~1956), nút "Mark Point" (~605) và `_mark_live_waypoint` (~1798) |
| `Software/New_GUI/serial_worker.py` | `FastCsvLogger._write_metadata_snapshot` (session_metadata.json, schema 5) |
| `Software/New_GUI/tools/diag_summary.py` | Tổng hợp mất gói cho cả phiên (không theo thời gian) |

Quy ước dấu bias (firmware `CMD_SET_DS_CAL` và `AnchorConfig.bias_mm`): **đo = thật + bias**, hiệu chỉnh = đo − bias.

### 1.2 Khung code và test mới (đã tạo, chưa có nội dung)

| File | Trạng thái |
|---|---|
| `Software/New_GUI/multi_spot_calibration.py` | Khung: dataclass đầy đủ, hàm `raise NotImplementedError`, docstring là đặc tả thuật toán |
| `Software/New_GUI/tools/radio_loss_report.py` | Khung + đặc tả |
| `Software/New_GUI/tools/static_dataset_report.py` | Khung + đặc tả |
| `Software/New_GUI/tools/dataset_manifest_example.json` | Manifest mẫu cho G0.3 |
| `Software/New_GUI/tests/g0_fixtures.py` | **Hoàn chỉnh.** Dữ liệu giả lập dùng chung (rig 5,4×3,6 m, bias thật, phiên log schema 5) |
| `tests/test_g0_multi_spot_solver.py` (12 test) | Hợp đồng bộ giải |
| `tests/test_g0_calibration_engine.py` (9 test) | Hợp đồng tích hợp `UwbStateEngine` + metadata |
| `tests/test_g0_calibration_gui.py` (5 test) | Hợp đồng giao diện |
| `tests/test_g0_radio_loss_report.py` (8 test) | Hợp đồng công cụ G0.2 |
| `tests/test_g0_static_dataset_report.py` (5 test) | Hợp đồng công cụ G0.3 |

Ngưỡng trong test đã được kiểm bằng một lời giải tham chiếu riêng (không đưa vào repo). Mọi test đều đạt được với thuật toán mô tả trong docstring, và có biên an toàn. Ví dụ: tham chiếu đạt 23 mm khi ngưỡng là 40 mm.

---

## 2. Bước 0 — chuẩn bị nhánh (người dùng xác nhận trước)

`Software/New_GUI/` hiện **chưa được git theo dõi** (untracked). Để review có diff sạch:

1. Tạo nhánh mới, ví dụ `feature/g0-calibration-dataset`, từ nhánh hiện tại.
2. Commit "baseline" gồm `Software/New_GUI/` ở trạng thái hiện tại, cộng khung code và test G0, **trước khi** Codex sửa gì. Không commit thư mục `logs/` (dung lượng lớn): thêm `Software/New_GUI/logs/` vào `.gitignore` nếu chưa có.
3. Codex làm việc trên nhánh này, mỗi mục G0.x một commit (hoặc vài commit nhỏ).

---

## 3. G0.1 — Hiệu chuẩn đa điểm

### 3.1 Giải thích ngắn

Một điểm đặt TAG chỉ cho 8 phương trình (8 anchor), nhưng có 11 ẩn: 8 bias và 3 tọa độ thật của điểm. Phần mềm cũ coi tọa độ gõ tay là đúng tuyệt đối, nên mọi sai số đặt điểm bị đổ hết vào bias.

Với ≥ 2 điểm, bài toán có hạng đủ. Ví dụ 4 điểm cho 32 phương trình và 20 ẩn. Khi đó tọa độ gõ tay chỉ còn là "gợi ý" (prior). Bộ giải ước lượng được bias, tìm ra điểm nào bị gõ sai, và báo lại cho người dùng.

Kiểm chứng trên rig 5,4×3,6 m: hạng (rank) / số ẩn là 8/11 với 1 điểm, 14/14 với 2 điểm, 20/20 với 4 điểm.

### 3.2 Bộ giải — `multi_spot_calibration.py`

Đặc tả đầy đủ nằm trong docstring của file. Tóm tắt:

- `observations_from_captures(captures, enabled_anchor_ids)`: chuyển `CalibrationSpotCapture` thành `SpotObservation` / `TapeObservation`, bỏ kết quả `LOW_SAMPLES` và anchor bị tắt.
- `solve_joint_calibration(anchors, spots, tapes, dimension, settings)`:
  - Gauss–Newton có trọng số Huber, tính lại ở mỗi vòng lặp.
  - Huber áp cho cả hàng dữ liệu **và** hàng prior của vị trí điểm. Nhờ vậy một điểm gõ sai 0,65 m không kéo lệch lời giải.
  - Prior rất yếu trên bias.
  - σ của mỗi hàng = √(sai số trung vị² + σ_hệ thống²), với σ_hệ thống mặc định 20 mm.
  - Hiệp phương sai được nhân với max(1, χ²/bậc tự do).
  - Chế độ 2D: z của điểm giữ nguyên giá trị gõ tay.
- **Quy tắc chọn phương pháp:** `"failed"` khi không có dữ liệu; `"single_spot_fallback"` khi có < 2 điểm (trả đúng số của phương pháp cũ); `"joint"` khi có ≥ 2 điểm.
- **Mã cảnh báo cố định:** `NO_DATA`, `SINGLE_SPOT`, `SPOT_FEW_ANCHORS`, `NOT_IDENTIFIABLE`, `SPOT_OFFSET`, `RESIDUAL_OUTLIER`, `RESIDUAL_RMS`, `BIAS_LARGE`, `FLAT_HEIGHTS`, `ANCHOR_ONE_SPOT`.
- `data_identifiability(...)`: hạng và số điều kiện (condition number) của Jacobian chỉ gồm dữ liệu.
- `suggest_calibration_spots(anchors, count)`: gợi ý 4 điểm ở 25%/75% kích thước hộp anchor theo X/Y, và 2 độ cao.
- `JointCalibrationResult.to_dict()` phải là JSON chặt (`allow_nan=False`).

### 3.3 Tích hợp vào `UwbStateEngine` (`core_state.py`)

Thêm (tên phải đúng vì test dùng):

```python
CALIBRATION_METHODS = ("joint", "median")
JOINT_WARNING_CODES = ("SPOT_OFFSET", "RESIDUAL_OUTLIER", "RESIDUAL_RMS", "BIAS_LARGE", "NOT_IDENTIFIABLE")

class UwbStateEngine:
    # in __init__ (next to self.spot_captures; NOT in reset_live_state):
    calibration_method: str = "joint"
    joint_settings: JointCalibrationSettings = JointCalibrationSettings()
    last_applied_calibration: dict | None = None

    def set_calibration_method(self, method: str) -> None: ...      # ValueError if not in CALIBRATION_METHODS
    def joint_calibration(self) -> JointCalibrationResult | None: ...
        # None when there is no capture. Uses enabled anchors + solver_dimension.
        # Cached: same object until captures (by identity), anchor x/y/z/enabled,
        # solver_dimension or joint_settings change.
    def active_calibration_method(self) -> str: ...
        # "joint" only if calibration_method == "joint" and the joint result is ok,
        # method == "joint" and identifiable; otherwise "median".
    def remove_capture(self, spot_index: int) -> bool: ...
    def calibration_summary(self) -> dict: ...
        # {"method_selected", "method_applied",
        #  "aggregated": {"<aid>": {"bias_mm", "sigma_mm", "captures"}},
        #  "joint": result.to_dict() | None}   -- strict JSON
```

Thay đổi hành vi:

- `aggregated_calibration()`: giữ nguyên kiểu trả về `{aid: (bias_mm, sigma_mm, count)}`.
  - Khi `active_calibration_method() == "joint"`: lấy bias từ lời giải chung; `sigma_mm` = σ mẫu trung bình như cũ (sàn 10 mm, hoặc 30 mm nếu thiếu); `count` = `spots_used + tape_used`.
  - Ngược lại: **giữ nguyên code trung vị cũ**. Tách code cũ thành `_median_aggregated_calibration()`.
- `calibration_warnings()`: giữ các cảnh báo cũ theo từng capture. Thêm `"Joint solve: <message>"` cho các mã trong `JOINT_WARNING_CODES`, chỉ khi `calibration_method == "joint"` và lời giải chung có `method == "joint"`. Với một điểm đúng, danh sách phải vẫn rỗng như cũ (đã có test).
- `apply_calibration_to_host()`: như cũ, thêm ghi `last_applied_calibration`:

  ```python
  {"applied_unix_s": time.time(), "method": "joint" | "median",
   "version": "multi-spot-joint-v1" | "per-spot-median-v1",
   "biases_mm": {"<aid>": float}, "spots": <number of spot captures>,
   "joint": result.to_dict() | None}
  ```
- `serial_worker.FastCsvLogger._write_metadata_snapshot`: thêm khóa `"calibration": eng.last_applied_calibration`.

### 3.4 Giao diện (`main_gui.py`, trang Calibration)

Tên widget phải đúng vì test dùng:

| Widget | Hành vi |
|---|---|
| `combo_cal_method` (QComboBox) | Mục 0: "Joint multi-spot (recommended)", data `"joint"`. Mục 1: "Median per spot (legacy)", data `"median"`. Đổi lựa chọn thì gọi `engine.set_calibration_method` |
| `btn_suggest_spots` | Gọi `suggest_calibration_spots(enabled anchors, 4)`, hiện danh sách trong `lbl_spot_plan`, nạp điểm đầu tiên vào `spin_cal_x/y/z` (nên ưu tiên điểm chưa được chụp) |
| `btn_remove_spot` | Xóa capture của dòng đang chọn trong `cal_spot_table` (`engine.remove_capture(spot_index)`) rồi cập nhật trang |
| `lbl_spot_plan` (QLabel) | Danh sách điểm gợi ý |
| `lbl_joint_status` (QLabel) | Khi < 2 điểm: câu có chữ số **2**, ví dụ "Joint solve needs at least 2 spot captures (have 1) — using per-spot median". Khi đã giải: có chữ **rank**, ví dụ "Joint: 4 spots · rank 20/20 · cond 10 · RMS 3 mm" |
| `cal_spot_table` (QTableWidget, 6 cột) | `#`, `Entered XYZ (m)`, `Estimated XYZ (m)`, `Offset (cm)`, `Resid RMS (mm)`, `Outliers`. Mỗi spot capture một dòng. Item cột 0 lưu `spot_index` ở `Qt.ItemDataRole.UserRole`. Chọn theo dòng. Tô đỏ dòng có `SPOT_OFFSET` |

Ngoài ra:

- Cột "Agg Bias (mm)" của bảng cũ: thêm ký hiệu phương pháp đang áp dụng. Ví dụ `+243.1 ±6 J` cho joint, `+243.1 (4x) M` cho median.
- Export JSON (`_export_calibration_records_dialog`): thêm khóa `"calibration_summary": engine.calibration_summary()`.
- Không gọi `_persist_rig_config` ở chỗ mới nào.

---

## 4. G0.2 — Công cụ chẩn đoán mất gói radio

**File:** `tools/radio_loss_report.py`. Đặc tả trong docstring.

```powershell
cd "Software\New_GUI"
python tools\radio_loss_report.py logs\uwb_fast_20261107_131601_COM14 --bin-s 5
```

Báo cáo gồm:
- Theo từng anchor: tỷ lệ được poll, tỷ lệ có range, tỷ lệ có RESP (từ `rx_quality.jsonl`), tỷ lệ trả lời muộn, số lần mất ≥ 0,2 s và lần mất dài nhất.
- Histogram số anchor mỗi chu kỳ.
- Số chu kỳ mất do UART (nhảy `cycle_seq`).
- Bin thời gian 5 s kèm nhiệt độ và điện áp TAG.
- Tương quan giữa nhiệt độ và chất lượng radio.
- Kết luận dạng lỗi:
  - `anchor_specific`: chỉ 1–2 anchor lỗi → kiểm nguồn, anten, đường nhìn thẳng của anchor đó.
  - `common_cause`: nhiều anchor lỗi **cùng chu kỳ** → nghi phía TAG (nguồn, nhiệt độ, timing) hoặc vật chắn gần TAG.
  - `independent`: nhiều anchor lỗi **độc lập** → nghi biên độ RF (khoảng cách, TX power, nhiễu).
  - `mixed`: tương quan một phần.

**Yêu cầu thêm cho Codex:** chạy công cụ trên `logs/uwb_fast_20261107_123110_COM14` và `logs/uwb_fast_20261107_131601_COM14`, rồi dán phần tóm tắt vào mô tả PR. Ghi chú 07/11 nghi lỗi "đều ở mọi anchor" (dạng `common_cause`); kết quả này sẽ xác nhận hoặc bác bỏ.

---

## 5. G0.3 — Bộ dữ liệu chuẩn

### 5.1 Code

1. **Nhãn waypoint trong GUI** (`main_gui.py`): thêm `edit_waypoint_label` (QLineEdit, placeholder "label (e.g. P1_start)") cạnh nút "Mark Point". `_mark_live_waypoint` gọi `engine.mark_waypoint(self.edit_waypoint_label.text())`; ô trống thì dùng nhãn mặc định `W<n>` như cũ.
   - Nên có (không bắt buộc): sau khi đánh dấu `X_start`, tự đổi ô thành `X_end`.
2. **Công cụ thống kê** `tools/static_dataset_report.py`, đặc tả trong docstring:

   ```powershell
   python tools\static_dataset_report.py logs\<session> tools\dataset_manifest_example.json
   ```

   - Mỗi đoạn tĩnh, mỗi anchor: số mẫu, % thành công, trung vị, σ, σ theo MAD, τ (thời gian tương quan), khoảng cách thật, bias thô, và **phần dư sau hiệu chuẩn** = trung vị − bias đang dùng − khoảng cách thật.
   - Mỗi đoạn: sai số vị trí so với điểm đã khảo sát.
   - Nghiệm thu (exit code 0/1):
     - mọi anchor có |phần dư| ≤ 50 mm;
     - sai số vị trí ≤ 10 cm;
     - % thành công trung bình ≥ 87,5% (7/8 anchor).

### 5.2 Quy trình ghi dữ liệu (người dùng làm với phần cứng)

Chuẩn bị:
- Tripod giữ TAG ở **đúng tư thế khi gắn trên drone** (anten dựng đứng, cùng hướng).
- Đo tọa độ tâm anten bằng laser hoặc thước, từ hai bức tường và từ sàn. Ghi vào manifest.
- Dùng nút **Suggest spots** để chọn P1–P4 (điểm hiệu chuẩn).
- Chọn thêm **V1, V2 là điểm kiểm chứng không dùng để hiệu chuẩn**. Bắt buộc, để nghiệm thu khách quan.

| Bước | Làm gì | Nhãn waypoint | Thời lượng |
|---|---|---|---|
| D0 | Bật Log CSV và RX_QUALITY/DIAG. Ghi rig hiện tại (`last_rig_config.json`) | – | – |
| D1 | **Hiệu chuẩn:** chụp 4 điểm P1–P4 trong trang Calibration (Samples ≥ 1000), xem bảng Spots, xử lý cảnh báo, Apply to Host | – | ~5 s mỗi điểm |
| D2 | **Đoạn tĩnh tại P1–P4 và V1, V2:** người đứng xa, không chắn đường TAG–anchor | `P1_start` … `P1_end`, …, `V2_start` … `V2_end` | ≥ 60 s mỗi điểm |
| D3 | Cầm TAG đi vòng chữ nhật trong phòng ở độ cao bay, quay về điểm xuất phát | `LOOP_start` … `LOOP_end` | 2–3 vòng |
| D4 (tùy chọn) | Xoay TAG tại V1 theo 8 hướng 0°, 45°, …, 315° | `R0_start` … `R7_end` | 15 s mỗi hướng |
| D5 (tùy chọn) | Đứng chắn lần lượt đường tới A1…A8 khi TAG ở V1 | `N1_start` … `N8_end` | 10 s mỗi anchor |
| D6 | **Chẩn đoán radio:** TAG đứng yên tại V1, không ai đi lại | – | 10 phút |

Sau khi ghi:
- Chạy `static_dataset_report.py` cho D2 (manifest có P1–P4, V1, V2).
- Chạy `radio_loss_report.py` cho D6.
- Giữ nguyên thư mục log (CSV + metadata + rx_quality + diag). Đây là bộ dữ liệu chuẩn cho G2.

### 5.3 Tiêu chí xong G0

| Tiêu chí | Ngưỡng | Đo bằng |
|---|---|---|
| Phần dư sau hiệu chuẩn tại **V1, V2** | mọi anchor ≤ 50 mm | `static_dataset_report.py` |
| Sai số vị trí tại V1, V2 | ≤ 10 cm | như trên |
| Lời giải hiệu chuẩn | `method = joint`, rank đủ, không còn `SPOT_OFFSET` chưa xử lý | trang Calibration |
| Radio khi LOS, đứng yên 10 phút | trung bình ≥ 7/8 anchor mỗi chu kỳ | `radio_loss_report.py` |
| Toàn bộ test | 117/117 xanh | `unittest` |

Nếu radio chưa đạt 7/8: G0 vẫn đóng phần code, còn nguyên nhân phần cứng ghi vào báo cáo và xử lý song song với G1.

---

## 6. G0.4 (tùy chọn, chỉ làm khi chủ dự án đồng ý)

Mục tiêu: firmware TAG gửi thêm chỉ số đỉnh CIR (`LDE_PPINDX`, có thể thêm `LDE_PPAMPL`) cho từng anchor trong gói `RX_QUALITY 0x19`.

Lý do: host tính được prNLOS theo Decawave APS006 Part 3, với IDiff = |FP_INDEX − PEAK_INDEX|:
- IDiff ≤ 3,3 → 0;
- 3,3 < IDiff < 6 → 0,39178·IDiff − 1,31719;
- IDiff ≥ 6 → 1.

Chỗ cần sửa:
- Đóng gói gói `0x19` trong `Firmware/common/telemetry/telemetry.c` (khoảng dòng 476–535).
- Parser `decode_rx_quality` trong `Software/UWB_UART_GUI/telemetry_protocol.py`. Parser hiện **kiểm độ dài chặt**, nên phải tăng version layout và chấp nhận cả độ dài cũ lẫn mới.

Làm trên nhánh riêng, sau G0.1–G0.3. Cần nạp firmware và thử phần cứng. Không thuộc tiêu chí xong G0.

---

## 7. Quy tắc cho Codex

1. **Không sửa** `tests/test_g0_*.py` và `tests/g0_fixtures.py`. Nếu thấy test sai, dừng lại và ghi lý do trong PR, không tự nới ngưỡng.
2. 78 test cũ phải xanh. Không thay đổi hành vi cũ ngoài những gì mục 3–5 yêu cầu. Riêng phương pháp "median" phải cho đúng số như trước.
3. Không thêm thư viện mới (chỉ PyQt6, numpy, pyserial và thư viện chuẩn).
4. `multi_spot_calibration.py` không import `core_state` và không import Qt.
5. Đóng mọi file đã mở (dùng `with`). Báo cáo JSON phải chặt (`allow_nan=False`).
6. Docstring và thông báo trong code viết bằng tiếng Anh, như phần còn lại của New_GUI.
7. Cập nhật `Software/New_GUI/README.md`: thêm mục "Joint multi-spot calibration" và "G0 tools" (cách chạy, ý nghĩa cảnh báo).
8. Chạy test:

   ```powershell
   cd "Software\New_GUI"
   $env:QT_QPA_PLATFORM = "offscreen"
   python -m unittest discover -s tests -p "test_*.py"
   ```

9. Không chạy GUI thật trên cổng COM. Không ghi đè `last_rig_config.json` (test GUI dùng `start_demo=False` và không bấm Apply).
10. Commit theo mục: G0.1 solver, G0.1 engine, G0.1 GUI, G0.2, G0.3, README.

---

## 8. Claude sẽ review những gì

1. `git diff` của `tests/test_g0_*.py` và `tests/g0_fixtures.py` phải rỗng.
2. 117/117 test xanh trên máy người dùng (Windows, PowerShell).
3. So bộ giải của Codex với lời giải tham chiếu riêng bằng Monte Carlo (200 lần ngẫu nhiên):
   - sai số bias;
   - khả năng phát hiện điểm gõ sai;
   - không có cảnh báo nhầm.
4. Đọc code: đúng đặc tả trong docstring (Huber trên prior điểm, chế độ 2D giữ z, xử lý dữ liệu suy biến, cache được làm mới đúng lúc).
5. Chạy hai công cụ trên các log thật trong `logs/`. Kiểm kết luận có hợp lý so với ghi chú 07/11 không.
6. Mở GUI ở chế độ demo: trang Calibration hiển thị đúng, các nút hoạt động, không lỗi khi chưa có capture.
7. README được cập nhật và không có thay đổi lạc chỗ ngoài phạm vi G0.

---

## 9. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/G0_KE_HOACH_HIEU_CHUAN_DA_DIEM_VA_DU_LIEU_CHUAN.md`. Làm mục G0.1, G0.2, G0.3 (bỏ G0.4) trong `Software/New_GUI`. Hợp đồng là các test `tests/test_g0_*.py` (39 test đang đỏ); đặc tả chi tiết nằm trong docstring của `multi_spot_calibration.py`, `tools/radio_loss_report.py` và `tools/static_dataset_report.py`. Không sửa các file test và `tests/g0_fixtures.py`. Giữ 78 test cũ xanh. Chạy `python -m unittest discover -s tests -p "test_*.py"` với `QT_QPA_PLATFORM=offscreen`. Commit theo từng mục trên nhánh `feature/g0-calibration-dataset`. Cuối cùng chạy `tools/radio_loss_report.py` trên `logs/uwb_fast_20261107_123110_COM14` và `logs/uwb_fast_20261107_131601_COM14`, dán kết quả vào mô tả thay đổi.
