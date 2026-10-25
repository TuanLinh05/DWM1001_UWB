# Phục hồi FINAL sau log 20261019_143252_COM14

## Kết luận từ log thật

TAG `0x394ce68f`, base450/slot250/margin350 UUS, boot duy nhất. Đây là
regression của bản pipeline: lịch radio bị rút quá sát trước khi có số đo
trên board chứng minh đủ thời gian. Việc tính DS được đưa sang sau khi arm
FINAL không làm giảm chi phí SPI cần trả trước lệnh delayed TX.

Trong cửa sổ DIAG 60,087 s:

| Counter | Mức tăng |
|---|---:|
| Cycle | 15.041 |
| FINAL bị từ chối vì trễ | 12.829 |
| FINAL chờ TXFRS hết hạn | 1.777 |
| Thiếu Rb cho phép đo DS | 116.460 |
| SPI error, radio recovery, config mismatch | 0 |
| UART TX overflow, measurement/burst queue drop | 0 |

Khoảng97,11% cycle không hoàn thành FINAL. RESP vẫn tới gần đầy đủ; timeout
RESP từng anchor chỉ tăng26–138 lần trong cùng cửa sổ. Do đó lỗi chính ở
FINAL, không phải toàn bộ anchor đồng loạt mất điện hoặc bị GUI loại số đo.

Đọc lại raw capture bằng decoder hiện tại: 15.303 burst, timestamp TAG trải
61,534045 s; khoảng249 cycle/s nhưng chỉ5,09–7,12 DS/s từng anchor và4,49
bộ đủ tám DS/s. Gap đo thật tới1,353 s ở A8. Có19 lỗi CRC trong thống kê
session, mất khoảng0,12% byte; cần tiếp tục theo dõi UART sau khi RF ổn,
nhưng mức này không giải thích việc97% cycle mất FINAL. Raw capture có thể
đã qua lớp ghi log; counter parser đọc lại không thay thế counter UART live.

Audit máy đọc được: `release/highrate_300/audit_log_20261019_143252.json`.
Không dùng `ds_ok=0` làm số mẫu DS vì profile chưa có DS calibration;
đếm RADIO_OK cùng mode DS trong RANGE_BURST, như các phiên trước.

## Sửa trong bản phục hồi

- Khôi phục base450/slot275/margin450 UUS, gap180 us, info extra64 UUS.
  Chính lịch radio này đã chạy double RX trong log134918 với98,63–99,78%
  DS thành công/cycle từng anchor. Thời gian POLL→FINAL đủ tám anchor trở
  về2825 UUS. Period3100 us vẫn chỉ là cận dưới, không phải tần số thực.
- Giữ pipeline tính range, double RX và periodic DIAG của vòng trước.
  Chưa có số đo hardware cho tổ hợp bản phục hồi này.
- Profile300 từ chối SET_BURST có slot<275 hoặc margin<450 UUS. Từ chối
  trước khi thay đổi timing đang chờ; compile cũng chặn default dưới cận
  profile. Các profile200/normal giữ cận command trước đây.
- Không thay đổi PHY hoặc công thức DS. Anchor300/PHY64 đã nạp vẫn dùng
  được; chỉ cần cập nhật TAG trong lần thử này.

## Kiểm thử chống tái diễn

Simulator trước đây coi SPI hoàn thành tức thời. Thêm chế độ stress tùy
chọn:18 us overhead/transaction +1 us mỗi byte ở8 MHz. Đây là ngân sách
giả lập bảo thủ để kiểm nhánh deadline, không phải WCET đã đo của nRF.
Deadline arm trừ phần preamble FINAL. Test tái hiện250/350 bị HPDWARN;
275/450 qua với đủ tám anchor, A3 tắt, cycle info và A8 không phản hồi.

Test đã thất bại trên default250/350 trước sửa và qua sau sửa. Đồng thời
kiểm command không thể áp lại slot250/margin350, kết quả DS không bị xuất
trùng hoặc biến một cycle thiếu FINAL thành range hợp lệ. Các test IRQ,
buffer ownership, transaction wrap, telemetry và calibration cũ vẫn chạy.

Host tests và build không thay thế thử nghiệm radio. Không công bố bản này
đạt300 DS/s trước khi nhận log mới từ board.

Kết quả kiểm tra bản phát hành: 26 nhóm host test qua; GUI 87 test,
2 skip theo môi trường. Build đủ chín node qua và verifier đối chiếu
9/9 HEX với manifest, vector, giới hạn bộ nhớ, build/config và SPI profile.
Dry-run script nạp chọn đúng TAG mới; không truy cập programmer.
TAG SHA256: `5c15f12ecdf194790ef56d70ae908b387fc6f83ae326db422d1a6983035a5efe`.

## Gói nạp và kiểm tra trên board

Config TAG bản phục hồi: **`0x0b033006`**. Xem manifest/verification của gói.
Bản lỗi394ce68f giữ trong
`previous_20261019_before_final_recovery/`; baseline84e6c9b9 vẫn ở
`previous_20261019_before_pipeline/`.

Đóng GUI/relay đang giữ COM14, nạp riêng TAG:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Dùng tiếp `DWM1001_UWB_Ground_Control_300hz.exe` hiện có; không đổi GUI.
Bật đủ tám anchor như phiên mới, ghi60–120 s. Đối chiếu config mới và
DIAG base450/slot275/margin450, xem mức tăng FINAL late/timeout, DS/s theo
timestamp TAG từng anchor, đủ tám DS/s, gap và UART CRC. Sau khi thử ngắn
ổn mới đo10 phút. Chưa tự nạp thiết bị trong lượt sửa này.
