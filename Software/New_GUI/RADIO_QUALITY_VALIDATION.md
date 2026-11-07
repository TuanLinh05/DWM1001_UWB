# Đánh giá chất lượng tín hiệu — 07/11/2026

Đã bổ sung `radio-quality-v1` vào Robust Adaptive EKF. Bật mặc định bằng
**Signal Quality Weight** trong sidebar. Khởi động lại GUI đang mở để nạp mã mới.
Chỉ số **Quality** và hệ số **R x** nằm trong bảng anchor; di chuột lên điểm
để xem lý do, nguồn thông tin và tuổi dữ liệu. R lớn nghĩa là tin phép đo ít hơn.
Khi dùng Legacy, lớp trọng số mới không được áp dụng.

## Dữ liệu và cách xử lý

- RX-FP gap: giữ cơ chế NLOS trước đây, không nhân lặp cùng chỉ số vào R.
- Công suất first path phía TAG và anchor: hạ độ tin cậy khi yếu, chỉ áp ngưỡng
  công suất tuyệt đối khi có cờ firmware FPP_CORRECTED.
- FP_AMPL1/2/3 và STD_NOISE: tính tỷ số biên độ first path/nền nhiễu dạng dB.
  Đây là proxy, chưa phải phép đo SNR đã hiệu chuẩn.
- RXPACC và cờ RESP/FINAL: kiểm tra tính hợp lệ trước khi dùng diagnostics.
- Công suất phía anchor: đánh giá thêm lần thu FINAL; không giả định hai phía
  luôn có chất lượng giống nhau.
- Carrier ppm: phát hiện thay đổi đột ngột, không phạt một offset ổn định lớn.
- Late reply: thêm một mức giảm độ tin cậy nhỏ.

Các chỉ số cùng mô tả kênh vô tuyến có tương quan được kết hợp bằng mức phạt
lớn nhất. Mức phạt clock được giới hạn; toàn bộ hệ số radio tối đa 36 lần R
trong cấu hình mặc định. Hệ số giảm độ tin cậy tăng ngay khi xấu, phục hồi với
hằng số thời gian 0,25 s. Mẫu diagnostics quá 0,30 s, thuộc boot khác hoặc nằm
trong tương lai không được dùng cho lần đánh giá hiện tại.

Mô hình chỉ tăng variance của cập nhật EKF. Nó không nới NIS gate, không sửa
khoảng cách raw/corrected và không tự học bias từ chính nghiệm tọa độ. Bootstrap
IRLS cũng giảm trọng số nguồn radio xấu. Các ngưỡng là heuristic kỹ thuật có thể
điều chỉnh, chưa phải mô hình xác suất LOS hay calibration cho mọi môi trường.

FP_INDEX, carrier integrator thô, CIR_PWR, nhiệt độ, điện áp và trường môi trường
được giữ trong log để phân tích/calibration sau. Không áp công thức bù khoảng
cách chưa được kiểm chứng từ những trường này. Frame hiện tại không có toàn
bộ dạng sóng CIR.

## Log và replay

Schema 5 giữ các cột cũ và thêm điểm chất lượng, nhãn, lý do, danh sách chỉ số
đã dùng, tuổi diagnostics, proxy FP/noise, R radio thực sự áp dụng và cờ bật/tắt.
Các trường quality để trống khi anchor không có mẫu trong Burst đó.

`rx_quality.jsonl` lưu đầy đủ từng message RX_QUALITY và số dòng Burst đã được
nhận vào logger trước message đó. Replay đọc cả sidecar và phục hồi đúng thứ
tự nhận, kể cả khi diagnostics đến sau Burst cùng cycle. Không dùng diagnostics
đến sau để làm đẹp lại kết quả quá khứ. Giữ sidecar cạnh CSV khi sao chép log.
Metadata ghi cấu hình radio, phiên bản/ngưỡng mô hình, cờ thang công suất và
số dòng/message bị rơi. Các log cũ vẫn dùng được phần power/ppm hiện có.

## Kiểm thử

**44 tests passed** với:

```powershell
python -m unittest discover -s tests -p "test_*.py" -q
```

Ngoài các kiểm thử EKF, calibration và GUI trước đây, bổ sung kiểm tra:

- Missing data không tự tạo penalty hay xác nhận GOOD.
- Cờ hợp lệ, tuổi dữ liệu, boot khác và thứ tự message.
- Tín hiệu yếu, FP/noise thấp, đường truyền chiều ngược xấu, thay đổi clock.
- Không nhân đôi NLOS và không nới gate cho spike lớn.
- Healthy radio cho nghiệm giống hệt khi tắt weighting mới.
- Raw sidecar replay tái tạo toàn bộ chuỗi X với sai khác dưới 1e-12 m trong
  ca kiểm thử, kể cả diagnostics đến sau Burst.
- Bật/tắt trên GUI, schema CSV và bộ lọc cũ tiếp tục hoạt động.

## Mô phỏng có vị trí thật để đối chiếu

| Tình huống | Tắt lớp mới: RMSE 3D | Bật lớp mới: RMSE 3D |
|---|---:|---:|
| Hai anchor có bias 110 mm và first path yếu | 6,49 cm | 1,31 cm |
| Hai anchor có bias 110 mm, FP/noise thấp dù RX-FP nhỏ | 6,57 cm | 1,45 cm |

Đây là các tình huống được dựng với chất lượng radio suy giảm đi kèm bias,
nhằm kiểm tra cơ chế giảm ảnh hưởng của phép đo xấu. Chúng không chứng minh
mức cải thiện tương tự trên TAG thật hay mọi loại NLOS.

## Replay hai log thực

So sánh với EKF đã triển khai trước lớp radio-quality này, không phải chỉ so với
đầu ra GUI cũ. Xử lý toàn bộ 129.590 và 99.775 mẫu, không bỏ mẫu trước khi lọc.

| Chỉ số: EKF trước → EKF + quality | Log 08:17:49 | Log 08:29:55 |
|---|---:|---:|
| Mẫu mất nghiệm | 0 → 0 | 0 → 0 |
| Bước >90 mm, tính cả gap | 0 → 0 | 2 → 1 |
| Bước >90 mm trong chu kỳ <20 ms | 0 → 0 | 0 → 0 |
| Bước lớn nhất, tính cả gap | 32,14 → 28,18 mm | 149,40 → 116,64 mm |
| P99 bước | 14,86 → 14,09 mm | 13,97 → 13,89 mm |

Bước còn trên 90 mm nằm tại t=240,981 s, sau gap 210,942 ms. Bước lớn nhất
trong chu kỳ <20 ms lần lượt 28,18 mm và 53,98 mm. Thử tắt lớp mới trên 1.000
mẫu đầu log 08:17:49 cho tọa độ giống hệt baseline EKF trước đó.

Hai log cũ không có sidecar RX_QUALITY thô, nên chỉ kiểm tra được các chỉ số
power/ppm đã lưu. Các log cũng không có ground truth: những số liệu này đánh
giá tính liên tục và tương thích, không chứng minh sai số tọa độ tuyệt đối.
Cần ghi log mới có RX_QUALITY tại các vị trí đo trước để kiểm chứng toàn bộ
lớp chất lượng trên phần cứng thực.

Thời gian xử lý engine/mẫu trong lần replay, trung vị/P95: log đầu
0,985/1,792 ms; log sau 1,822/4,248 ms. Đây không phải latency end-to-end qua
UART/render, và không bảo đảm mọi mẫu đều xử lý trong một chu kỳ đo.

## Sử dụng

1. Khởi động lại GUI, chọn Robust Adaptive EKF và giữ Signal Quality Weight bật.
2. Kết nối TAG. Nếu chưa có diagnostics, dùng Enable RX_QUALITY ở Rig Layout &
   Diag; tooltip Quality sẽ cho biết diagnostics có đang được dùng hay không.
3. Bật Log CSV; giữ cả CSV, metadata và rx_quality.jsonl.
4. Ghi các đoạn đứng yên tại vị trí/độ cao đã đo trước, rồi đoạn di chuyển.
5. Re-solve với checkbox quality bật/tắt để đối chiếu trên cùng dữ liệu.

CLI:

```powershell
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 --output radio_validation --signal-quality on
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 --output radio_validation --signal-quality off
```
