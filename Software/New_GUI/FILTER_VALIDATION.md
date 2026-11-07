# Kết quả sửa bộ lọc — 07/11/2026

GUI mặc định dùng **Robust Adaptive EKF**. Có thể chọn **Legacy CV + IRLS**
trong sidebar để đối chiếu. Cần khởi động lại GUI đang mở để nạp mã mới.

## Thay đổi

- EKF trạng thái XYZ/vận tốc, cập nhật trực tiếp từng khoảng cách đã hiệu chỉnh.
  R thích nghi theo nhiễu từng anchor, NLOS và lịch sử chất lượng. Innovation
  gate 5 sigma loại ngoại lai lớn; Huber giảm trọng số ngoại lai vừa.
- Dùng thời gian TAG cho lọc, vận tốc và cửa sổ chuyển động. Khử trùng Burst;
  không đưa RANGE_MEAS trùng vào bộ lọc khi Burst đang hoạt động.
- Sửa cửa sổ nhiễu bị kẹt sau thay đổi khoảng cách >500 mm, dùng MAD của sai phân.
  Luồng Legacy trả về kết quả CV thực tế và không chồng thêm EMA khi chuyển động.
- EKF không loại cứng anchor theo ngưỡng nhiễu 45 mm. Trọng số được giảm nhanh
  khi xấu và phục hồi từ từ khi tốt. Mẫu cũ không được tái sử dụng để cập nhật EKF.
- Mất full fix quá 0,35 s sẽ ngừng công bố nghiệm; sau gap >0,8 s cần khởi tạo lại.
  Đổi cấu hình anchor, chiều giải hoặc reboot sẽ reset trạng thái tương ứng.
- Replay xử lý mọi mẫu trước khi giảm điểm hiển thị; chạy nền, có tiến độ và hủy
  khi đóng GUI. Log schema 4 bổ sung mode, covariance dạng độ lệch chuẩn,
  prediction age, cờ gốc, innovation, NIS và lý do loại phép đo.

## Replay dữ liệu thực

So sánh đầu ra đã ghi trong log với kết quả replay thuật toán mới. Mỗi dòng CSV
được xử lý, không bỏ mẫu trước khi lọc. Cả hai phiên đều là di chuyển bằng tay.

| Chỉ số | Log 08:17:49 | Log 08:29:55 |
|---|---:|---:|
| Số mẫu | 129.590 | 99.775 |
| Mẫu có nghiệm EKF | 129.590 / 129.590 | 99.775 / 99.775 |
| Bước >90 mm giữa hai mẫu, trước → sau | 143 → 0 | 167 → 2 |
| Bước >90 mm ở chu kỳ <20 ms, trước → sau | 143 → 0 | 167 → 0 |
| Bước lớn nhất, tính cả gap, trước → sau | 267,4 → 32,1 mm | 577,3 → 149,4 mm |
| Bước EKF lớn nhất ở chu kỳ <20 ms | 32,1 mm | 54,0 mm |
| P99 bước, trước → sau | 35,1 → 14,9 mm | 33,3 → 14,0 mm |
| Trung vị thời gian engine/mẫu trong lần benchmark | 1,379 ms | 1,631 ms |
| P95 thời gian engine/mẫu | 3,128 ms | 4,084 ms |

Hai bước còn trên 90 mm ở log thứ hai nằm tại:

- t=240,981 s: 149,4 mm sau gap 210,942 ms.
- t=329,803 s: 99,8 mm sau gap 260,154 ms.

Các gap này dài hơn nhiều so với chu kỳ thường khoảng 3,9 ms; không thể coi
chúng như bước nhảy trong một chu kỳ 4 ms. Số lần nhảy giảm không đến từ việc
bỏ nghiệm: cả hai phiên có nghiệm cho mọi dòng, và không có cặp đầu ra XYZ
liên tiếp hoàn toàn giống nhau trong replay.

Thời gian trên đo riêng engine trên máy hiện tại khi replay, không phải đo độ
trễ end-to-end qua UART và render. P99 lần lượt 4,629 và 6,317 ms; có đuôi xử lý
dài do tải hệ thống/solver, nên không cam kết mọi mẫu luôn xử lý dưới một chu kỳ.

## Kiểm thử

`python -m unittest discover -s tests -p "test_*.py" -q`: **30 tests passed**.

Bao gồm tương thích firmware, calibration, nghiệm 2D/3D, chuyển động có vị trí
thật trong mô phỏng, chuyển động chậm, spike 27 m, NLOS kéo dài, mất anchor,
gap, reboot, trùng/đảo thứ tự mẫu, bất biến khi thay cách chia batch host,
covariance, replay đủ mẫu, dấu bias firmware/host, độ rộng CSV khi mất nghiệm,
chọn bộ lọc trong GUI và replay nền. Đường mô phỏng có nhiễu/NLOS/spike đạt
ngưỡng RMSE 3D dưới 6,5 cm và bước dưới 40 mm theo kiểm thử.

Đã chạy GUI với simulator và xác nhận có nghiệm. Chưa kiểm tra trên TAG thật
trong phiên làm việc này. Hai log thực không có ground truth: các số trên chứng
minh cải thiện tính liên tục, chưa chứng minh sai số tọa độ tuyệt đối. Covariance
cũng phụ thuộc mô hình nhiễu; không phải bảo đảm độ chính xác thực tế.

## Chạy lại

```powershell
python -m pip install -r requirements.txt
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 logs/uwb_fast_20261107_082955_COM14 --output filter_validation
```

Nguồn chính: `robust_range_ekf.py`, tích hợp trong `core_state.py`, lựa chọn GUI
trong `main_gui.py`, log và replay worker trong `serial_worker.py`.
