# G0.1b — Giải lượt hai: bỏ prior của điểm gõ sai

**Ngày:** 08/11/2026 · **Code:** Codex · **Review:** Claude
**Nhánh:** tiếp tục trên `feature/g0-calibration-dataset` (chưa merge)
**Hợp đồng:** `Software/New_GUI/tests/test_g0b_spot_prior_release.py` — **15 test đang đỏ**. Không sửa file này.
**Đọc trước:** `Plan/G0_KE_HOACH_HIEU_CHUAN_DA_DIEM_VA_DU_LIEU_CHUAN.md` (G0.1) và docstring `solve_joint_calibration`.

---

## 1. Vì sao cần

Bộ giải G0.1 coi tọa độ gõ tay là prior có trọng số Huber. Khi một điểm bị gõ sai, Huber chỉ giảm tác động của prior đó, không loại bỏ hẳn. Phần prior còn lại vẫn kéo lệch vị trí các điểm khác và các bias khoảng 1–2 cm. Có thể thấy trên ảnh chụp trang Calibration: A1 ra +44,8 mm, giá trị đúng là +35.

Bản thử của người review (cùng thuật toán như mô tả dưới đây) cho kết quả:

| Kịch bản (sai số bias lớn nhất) | Hiện tại (1 lượt) | Có lượt hai |
|---|---:|---:|
| Dữ liệu chuẩn, P2 gõ lệch 0,65 m | 10,5 mm | **0,02 mm** |
| Nhiễu giả lập (fixture), P2 gõ sai | 14,5 mm | **4,0 mm** |
| P2 và P4 cùng gõ sai | 27,7 mm | **4,5 mm** |
| Chỉ 2 điểm, P2 gõ sai | 55,8 mm | **6,1 mm** |
| Monte Carlo 200 lần (nhiễu 5 mm, điểm lệch 2 cm), 1 điểm sai: p95 | 21,7 mm | **14,1 mm** |
| Monte Carlo như trên, 2 điểm sai: p95 | 36,3 mm | **15,2 mm** |
| Monte Carlo như trên, dữ liệu sạch: p95 | 14,7 mm | 14,7 mm (không đổi) |
| Dữ liệu sạch (fixture) | 3,46 mm | 3,46 mm (không đổi, vì chỉ chạy 1 lượt) |

Trong 600 lần Monte Carlo trên, lần nào cũng bỏ prior đúng các điểm gõ sai, và không lần nào bỏ nhầm prior của điểm đúng.

Lượt hai **không được** che giấu lỗi hệ thống: nếu đa số điểm cùng lệch (sai gốc tọa độ rig, sai tọa độ anchor), phần mềm không bỏ prior nào mà báo `MANY_SPOT_OFFSETS`.

---

## 2. Đặc tả

### 2.1 `multi_spot_calibration.py`

**Cấu trúc dữ liệu** (thêm field có giá trị mặc định ở **cuối** dataclass để không phá chỗ gọi hiện có):

```python
SOLVER_VERSION = "multi-spot-joint-v2"          # bump: behaviour changed

WARNING_CODES += ("MANY_SPOT_OFFSETS",)        # add to the tuple

@dataclass(frozen=True)
class JointCalibrationSettings:
    ...
    release_flagged_spot_priors: bool = True    # run the release pass(es)
    max_released_fraction: float = 0.5          # release at most floor(fraction * n_spots) spots
    max_passes: int = 3                         # passes in total, including the first

@dataclass(frozen=True)
class JointSpotResult:
    ...
    residual_rms_mm: float | None               # None when every anchor of the spot is an outlier
    outlier_anchor_ids: tuple[int, ...]
    prior_released: bool = False

@dataclass(frozen=True)
class JointCalibrationResult:
    ...
    version: str = SOLVER_VERSION
    passes: int = 0                             # joint passes run; 0 for fallback/failed
    released_spot_indices: tuple[int, ...] = () # spot_index values, sorted
```

**Thuật toán** (thay khối Gauss–Newton hiện tại bằng vòng lặp theo lượt):

1. `R = {}` (các điểm đã bỏ prior), `passes = 0`.
2. Mỗi lượt: `passes += 1`, chạy Gauss–Newton như hiện tại, **khởi động từ θ của lượt trước**. Hàng prior vị trí của các điểm trong `R` **bị bỏ hẳn**, không chỉ giảm trọng số. Mọi hàng khác giữ nguyên, Huber tính lại như cũ.
3. Sau mỗi lượt, tính hiệp phương sai như hiện tại và tập cờ `F` = các điểm có `offset_m > max(spot_offset_warn_m, 3·offset_std_m)`. Đây cũng chính là tiêu chí `SPOT_OFFSET`.
4. Dừng nếu một trong các điều kiện sau đúng:
   - `release_flagged_spot_priors` là False;
   - `passes == max_passes`;
   - `F ⊆ R` (không có điểm mới bị cờ).
5. Nếu `|F ∪ R| > floor(max_released_fraction · n_spots)`: dừng, **giữ kết quả lượt hiện tại**, thêm cảnh báo `MANY_SPOT_OFFSETS`. Message **phải chứa cụm "anchor coordinates"**, ví dụ: "Most spots disagree with their entered coordinates; check anchor coordinates and the rig origin/axes. No spot prior was released."
6. Ngược lại: `R ← R ∪ F` rồi quay lại bước 2.
7. Mọi đầu ra (hiệp phương sai, phần dư, outlier, rank, cảnh báo) lấy từ **lượt cuối**.
   - `SPOT_OFFSET` tính lại cho mọi điểm. Điểm đã bỏ prior vẫn giữ cảnh báo này, và message thêm câu: "Its prior was released; position estimated from ranges only."
   - `iterations` = tổng số vòng Gauss–Newton của mọi lượt; `passes` = số lượt.
   - `released_spot_indices` = `spot_index` của các điểm trong `R`, sắp tăng dần; `JointSpotResult.prior_released = (điểm thuộc R)`.
8. Sửa lỗi nhỏ: `JointSpotResult.residual_rms_mm` là `None` khi mọi anchor của điểm đều là outlier. Hiện code trả `0.0` qua `_rms(good) or 0.0`.
9. Kiểm tra settings mới, sai thì xử lý như settings sai hiện có (`method="failed"`, `NOT_IDENTIFIABLE`):
   - `release_flagged_spot_priors` phải là `bool` thật;
   - `max_passes` là int ≥ 1 và không phải `bool`;
   - `max_released_fraction` hữu hạn, nằm trong [0, 1].
10. Kết quả fallback hoặc failed: `passes = 0`, `released_spot_indices = ()`.
11. Cập nhật docstring của `solve_joint_calibration` (thêm bước "release passes") và docstring đầu module.

### 2.2 `core_state.py`

- Thêm `"MANY_SPOT_OFFSETS"` vào `JOINT_WARNING_CODES`. Khi đó cảnh báo này hiện trong `calibration_warnings()` dạng `"Joint solve: …"`, và hộp xác nhận Apply/Write sẽ hỏi.
- `last_applied_calibration["joint"]` đã lấy từ `to_dict()`, nên tự có `passes`, `released_spot_indices`, `prior_released`. Kiểm lại cho chắc.

### 2.3 `main_gui.py`

- `lbl_joint_status`: khi có điểm bỏ prior, thêm `· priors released: #2, #4` (giữ chữ "rank" như cũ). Ví dụ: `Joint: 4 spots · rank 20/20 · cond 10.2 · RMS 2.4 mm · priors released: #2 · applying joint`.
- `cal_spot_table`, cột **Offset (cm)**: điểm bỏ prior hiện `63.3 (released)`, kèm tooltip "Prior released: position estimated from ranges only". Dòng vẫn tô đỏ như cũ vì vẫn có `SPOT_OFFSET`.
- Cột **Resid RMS (mm)**: hiện `--` khi `residual_rms_mm is None`. Code hiện tại sẽ lỗi `TypeError` khi format `None`.

### 2.4 Tài liệu

- `README.md`, mục "Joint multi-spot calibration":
  - thêm một đoạn về lượt hai và giới hạn `max_released_fraction`;
  - thêm dòng `MANY_SPOT_OFFSETS` vào bảng mã cảnh báo;
  - nói rõ: lượt hai là **lưới an toàn**, cách đúng vẫn là đo lại hoặc xóa điểm đỏ rồi chụp lại.
- `G0_CHANGE_DESCRIPTION.md`: thêm mục G0.1b (thay đổi, kết quả test, Monte Carlo trước/sau).

---

## 3. Quy tắc

1. Không sửa `tests/test_g0_*.py`, `tests/test_g0b_spot_prior_release.py`, `tests/g0_fixtures.py`.
2. 128 test hiện có phải xanh. `tests/test_calibration_regressions.py` (của Codex) chỉ được sửa nếu đặc tả mới bắt buộc, và phải ghi lý do trong commit.
3. Không thêm thư viện. Không ghi `last_rig_config.json`. Không mở cổng COM.
4. Chạy test:

   ```powershell
   cd "Software\New_GUI"
   $env:QT_QPA_PLATFORM = "offscreen"
   python -m unittest discover -s tests -p "test_*.py"
   ```

   Kỳ vọng: **143/143**.
5. Commit gợi ý:
   - `feat(g0.1b): release priors of mistyped spots in a second solve pass`
   - `fix(g0.1b): report missing per-spot residual RMS`
   - `feat(g0.1b): show released spots in the calibration page`
   - `docs(g0.1b): ...`

---

## 4. Claude sẽ review

1. `git diff` các file hợp đồng phải rỗng; 143/143 test xanh.
2. Monte Carlo so với bản thử của người review, cùng đầu vào. Sai khác bias kỳ vọng ≈ 0 vì cùng thuật toán. Các kịch bản: 1 điểm sai, 2 điểm sai, đa số sai, sai gốc rig, dữ liệu sạch.
3. Đọc code:
   - bỏ **hẳn** hàng prior, không chỉ giảm trọng số;
   - khởi động lượt sau từ θ của lượt trước;
   - giới hạn số lượt và tỷ lệ điểm được bỏ prior;
   - kiểm tra settings;
   - `residual_rms_mm` trả `None`.
4. Chụp trang Calibration ở theme sáng và tối với P2 gõ sai.
5. README và `SOLVER_VERSION = "multi-spot-joint-v2"`.

---

## 5. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/G0_1B_YEU_CAU_GIAI_LUOT_HAI.md` và làm trên nhánh `feature/g0-calibration-dataset`. Hợp đồng là `Software/New_GUI/tests/test_g0b_spot_prior_release.py` (15 test đang đỏ); không sửa các file test hợp đồng và `tests/g0_fixtures.py`. Thực hiện mục 2.1–2.4: lượt giải thứ hai bỏ hẳn prior của các điểm bị `SPOT_OFFSET` (tối đa `floor(0.5·n)` điểm, tối đa 3 lượt, khởi động từ lời giải trước), cảnh báo `MANY_SPOT_OFFSETS` khi đa số điểm lệch, `residual_rms_mm = None` khi mọi anchor của điểm là outlier, hiển thị trên trang Calibration, `SOLVER_VERSION = "multi-spot-joint-v2"`, cập nhật README và `G0_CHANGE_DESCRIPTION.md`. Chạy `python -m unittest discover -s tests -p "test_*.py"` với `QT_QPA_PLATFORM=offscreen`; kỳ vọng 143/143.
