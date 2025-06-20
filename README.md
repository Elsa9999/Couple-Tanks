# Couple-Tanks: Mô phỏng điều khiển bồn nước đôi bằng PID & Fuzzy PID

## Mục lục
- [Giới thiệu](#giới-thiệu)
- [Tính năng nổi bật](#tính-năng-nổi-bật)
- [Yêu cầu hệ thống](#yêu-cầu-hệ-thống)
- [Cài đặt](#cài-đặt)
- [Hướng dẫn sử dụng](#hướng-dẫn-sử-dụng)
- [Cấu trúc mã nguồn](#cấu-trúc-mã-nguồn)
- [Tài liệu liên quan](#tài-liệu-liên-quan)

---

## Giới thiệu

Ứng dụng này mô phỏng hệ thống hai bồn nước kết nối vật lý, sử dụng bộ điều khiển PID truyền thống và PID mờ (Fuzzy PID). Giao diện trực quan, dễ sử dụng, cho phép bạn:
- Quan sát mực nước, dòng chảy, trạng thái van, nhiễu loạn.
- Tinh chỉnh thông số PID, setpoint, độ mở van.
- Tự động tinh chỉnh PID bằng phương pháp relay (Ziegler-Nichols).
- Xuất dữ liệu mô phỏng ra file CSV.

## Tính năng nổi bật
- **Mô phỏng vật lý thực tế**: Hai bồn nước, van xả, dòng chảy, nhiễu loạn.
- **Điều khiển PID & Fuzzy PID**: Chuyển đổi linh hoạt giữa hai bộ điều khiển.
- **Auto-tuning PID**: Tự động tìm thông số tối ưu bằng relay method.
- **Hoạt họa dòng chảy**: Hình ảnh động trực quan, màu sắc phản ánh trạng thái.
- **Biểu đồ động**: Hiển thị mực nước thực tế và setpoint theo thời gian.
- **Giao diện thân thiện**: Các tab chức năng, thanh trượt, nút bấm rõ ràng.
- **Xuất dữ liệu**: Lưu kết quả mô phỏng ra file CSV.

## Yêu cầu hệ thống
- Python >= 3.7
- Thư viện: `matplotlib`, `numpy`, `tkinter` (có sẵn với Python chuẩn)

Cài đặt nhanh:
```bash
pip install matplotlib numpy
```

## Cài đặt
1. Clone hoặc tải mã nguồn về máy:
   ```bash
   git clone https://github.com/Elsa9999/Couple-Tanks.git
   cd Couple-Tanks
   ```
2. Cài đặt thư viện cần thiết:
   ```bash
   pip install matplotlib numpy
   ```

## Hướng dẫn sử dụng
1. Chạy chương trình:
   ```bash
   python coupled_tank_gui.py
   ```
2. Giao diện sẽ hiện ra với 3 tab:
   - **Vận hành**: Quan sát mô phỏng, điều khiển van, tạo nhiễu, xem biểu đồ.
   - **Tinh chỉnh PID**: Điều chỉnh Kp, Ki, Kd, setpoint, auto-tuning.
   - **Thiết lập Logic Mờ**: Xem bảng luật mờ của Fuzzy PID.
3. Các bước cơ bản:
   - Chọn bộ điều khiển (PID hoặc Fuzzy PID).
   - Điều chỉnh thông số PID hoặc để auto-tuning.
   - Đặt setpoint, độ mở van.
   - Nhấn **Bắt đầu** để chạy mô phỏng.
   - Nhấn **Tạo Nhiễu** để kiểm tra khả năng phục hồi.
   - Quan sát biểu đồ, hoạt họa dòng chảy.
   - Xuất dữ liệu bằng menu **Tệp > Xuất dữ liệu ra CSV...**

## Cấu trúc mã nguồn
- `coupled_tank_gui.py`: Toàn bộ mã nguồn chính, gồm các lớp:
  - `PIDController`: Bộ điều khiển PID.
  - `FuzzyPIDController`: Bộ điều khiển PID mờ (Mamdani).
  - `CoupledTankSystem`: Mô phỏng vật lý hai bồn nước.
  - `SimulationGUI`: Giao diện người dùng, hoạt họa, biểu đồ, xuất dữ liệu.
- `FUZZY_PID_DOCUMENTATION.md`: Giải thích chi tiết về Fuzzy PID.
- `RELAY_METHOD_DOCUMENTATION.md`: Giải thích chi tiết về phương pháp relay auto-tuning.
- `test_gui_improvements.py`: (Nếu có) Mã kiểm thử cải tiến giao diện.
- `fuzzy_pid_test_results.png`: Hình ảnh kết quả kiểm thử Fuzzy PID.

## Tài liệu liên quan
- [FUZZY_PID_DOCUMENTATION.md](./FUZZY_PID_DOCUMENTATION.md)
- [RELAY_METHOD_DOCUMENTATION.md](./RELAY_METHOD_DOCUMENTATION.md)

---

**Tác giả:** Elsa9999  
**Github:** https://github.com/Elsa9999/Couple-Tanks

Nếu có thắc mắc, vui lòng tạo issue trên Github hoặc liên hệ tác giả.
