# Tài liệu Bộ điều khiển Fuzzy PID

## Tổng quan

Lớp `FuzzyPIDController` triển khai bộ điều khiển logic mờ kiểu Mamdani, tự động điều chỉnh các hệ số PID (Kp, Ki, Kd) dựa trên sai số (E) và tốc độ thay đổi sai số (CE). Điều này giúp điều khiển thích nghi, xử lý tốt các hệ phi tuyến và điều kiện vận hành thay đổi, vượt trội hơn so với PID truyền thống với hệ số cố định.

## Kiến trúc

### 1. Biến ngôn ngữ (Linguistic Variables)
Bộ điều khiển sử dụng 7 thuật ngữ ngôn ngữ cho cả biến vào và ra:
- **NB**: Âm lớn (Negative Big)
- **NM**: Âm trung bình (Negative Medium)
- **NS**: Âm nhỏ (Negative Small)
- **ZO**: Không (Zero)
- **PS**: Dương nhỏ (Positive Small)
- **PM**: Dương trung bình (Positive Medium)
- **PB**: Dương lớn (Positive Big)

### 2. Miền giá trị (Universe of Discourse)
- **Sai số (E)**: -50 đến 50 cm (101 điểm rời rạc)
- **Tốc độ thay đổi sai số (CE)**: -20 đến 20 cm/s (81 điểm rời rạc)
- **Kp**: 0 đến 200 (101 điểm)
- **Ki**: 0 đến 50 (101 điểm)
- **Kd**: 0 đến 300 (101 điểm)

### 3. Hàm thành viên (Membership Functions)
Tất cả các hàm thành viên đều là tam giác, xác định bởi ba điểm (a, b, c):
- Với biến vào: phủ toàn bộ miền giá trị, các vùng chồng lấn
- Với biến ra: thiết kế để cung cấp dải hệ số phù hợp cho từng kịch bản điều khiển

## Quá trình suy luận mờ

### 1. Làm mờ (Fuzzification)
Chuyển giá trị đầu vào (crisp) thành mức độ thuộc về các thuật ngữ ngôn ngữ bằng phép nội suy tuyến tính.

### 2. Suy luận luật (Rule Inference)
Sử dụng phương pháp Mamdani với 49 luật (7×7):
- Độ mạnh luật = min(mức độ thuộc E, mức độ thuộc CE)
- Hàm thành viên đầu ra = min(độ mạnh luật, hàm thành viên đầu ra)
- Tổng hợp bằng toán tử max

### 3. Giải mờ (Defuzzification)
Dùng phương pháp trọng tâm (centroid) để chuyển kết quả mờ thành giá trị thực:
```
centroid = Σ(fuzzy_output × universe) / Σ(fuzzy_output)
```

## Thiết kế bảng luật

Bảng luật tuân theo nguyên tắc điều chỉnh PID mờ tiêu chuẩn:

### Ví dụ một số luật chính:
- **Sai số lớn, tốc độ thay đổi lớn**: Kp cao, Ki thấp, Kd trung bình
- **Sai số nhỏ, tốc độ thay đổi nhỏ**: Kp thấp, Ki trung bình, Kd thấp
- **Sai số 0, tốc độ thay đổi 0**: Kp, Ki, Kd trung bình

### Bảng luật đầy đủ:
```
(E, CE) → (Kp, Ki, Kd)
(NB, NB) → (PB, NB, PS)  # Sai số âm lớn, tốc độ thay đổi âm lớn
(NB, NM) → (PB, NB, NS)  # Sai số âm lớn, tốc độ thay đổi âm trung bình
...
(ZO, ZO) → (ZO, ZO, ZO)  # Sai số 0, tốc độ thay đổi 0
...
(PB, PB) → (PB, NB, PS)  # Sai số dương lớn, tốc độ thay đổi dương lớn
```

## Hướng dẫn sử dụng

### Sử dụng cơ bản
```python
# Tạo bộ điều khiển
controller = FuzzyPIDController(set_point=25.0, output_limits=(0, 300))

# Cập nhật bộ điều khiển
output = controller.update(process_variable, dt)

# Thay đổi setpoint
controller.set_setpoint(new_setpoint)

# Đặt lại bộ điều khiển
controller.reset()
```

### Tích hợp với hệ thống bồn nước đôi
Bộ điều khiển này thiết kế để tích hợp trực tiếp với mô phỏng bồn nước đôi:
- Giao diện tương thích với `PIDController`
- Giới hạn đầu ra phù hợp (0-300 cm³/s)
- Quy mô sai số phù hợp với điều khiển mực nước (0-40 cm)

## Đặc tính hiệu năng

### Ưu điểm:
1. **Điều khiển thích nghi**: Hệ số tự động điều chỉnh theo điều kiện sai số
2. **Xử lý phi tuyến**: Hiệu quả hơn với hệ phi tuyến
3. **Robustness**: Ít nhạy cảm với thay đổi tham số
4. **Dễ tinh chỉnh**: Dựa trên luật, dễ hiểu và điều chỉnh

### Hiệu năng điển hình:
- **Thời gian tăng**: Tương đương PID tối ưu
- **Vượt quá**: Thường thấp hơn PID cố định
- **Thời gian xác lập**: Thường nhanh hơn nhờ hệ số thích nghi
- **Sai số xác lập**: Được loại bỏ nhờ thành phần tích phân

## Hướng dẫn tinh chỉnh

### 1. Điều chỉnh hàm thành viên
- **Dải sai số**: Điều chỉnh theo biên độ sai số thực tế
- **Dải tốc độ thay đổi**: Điều chỉnh theo động học hệ thống
- **Dải đầu ra**: Điều chỉnh theo giới hạn cơ cấu chấp hành

### 2. Điều chỉnh bảng luật
- **Luật an toàn**: Bắt đầu với hệ số vừa phải
- **Luật mạnh**: Tăng hệ số cho đáp ứng nhanh (có thể gây vượt quá)
- **Ổn định**: Đảm bảo luật cung cấp đủ giảm chấn

### 3. Tối ưu hiệu năng
- **Thời gian tăng**: Tăng Kp ở các luật liên quan
- **Vượt quá**: Giảm Kp hoặc tăng Kd ở các luật liên quan
- **Thời gian xác lập**: Điều chỉnh Ki ở các luật liên quan

## Chi tiết triển khai

### Các phương thức chính:
- `_init_membership_functions()`: Tạo hàm thành viên tam giác
- `_init_rule_base()`: Định nghĩa 49 luật mờ
- `_fuzzify()`: Làm mờ giá trị đầu vào
- `_fuzzy_inference()`: Suy luận mờ kiểu Mamdani
- `_defuzzify()`: Giải mờ bằng phương pháp trọng tâm
- `update()`: Vòng lặp điều khiển chính tích hợp các bước mờ

### Bảo vệ chống tích phân quá mức (Anti-Windup):
- Thành phần tích phân bị giới hạn trong dải đầu ra
- Ngăn tích phân quá mức khi bão hòa

### Hiệu quả tính toán:
- Sử dụng numpy cho phép toán vector
- Làm mờ bằng nội suy tuyến tính
- Đánh giá luật hiệu quả với dừng sớm

## So sánh với PID truyền thống

| Tiêu chí | PID truyền thống | Fuzzy PID |
|----------|------------------|-----------|
| **Tinh chỉnh hệ số** | Cố định | Thích nghi |
| **Hệ phi tuyến** | Hiệu năng hạn chế | Hiệu năng tốt |
| **Công sức tinh chỉnh** | Cần chuyên môn | Dựa trên luật |
| **Chi phí tính toán** | Thấp | Trung bình |
| **Robustness** | Trung bình | Cao |
| **Triển khai** | Đơn giản | Phức tạp hơn |

## Định hướng phát triển

### Cải tiến tiềm năng:
1. **Bảng luật thích nghi**: Tự động cập nhật luật theo hiệu năng
2. **Fuzzy Type-2**: Xử lý bất định trong hàm thành viên
3. **Tối ưu hóa**: Dùng thuật toán di truyền, bầy đàn để tối ưu luật
4. **Đa mục tiêu**: Xem xét nhiều tiêu chí hiệu năng khi thiết kế luật

### Khả năng tích hợp:
1. **Điều khiển lai**: Kết hợp với các chiến lược điều khiển khác
2. **Điều khiển dự đoán**: Dùng logic mờ cho ràng buộc
3. **Mạng nơ-ron**: Logic mờ cho khởi tạo mạng nơ-ron

## Xử lý sự cố

### Vấn đề thường gặp:
1. **Dao động**: Giảm Kp ở các luật liên quan
2. **Đáp ứng chậm**: Tăng Kp ở các luật liên quan
3. **Sai số xác lập**: Kiểm tra thành phần tích phân và Ki
4. **Bão hòa**: Kiểm tra giới hạn đầu ra và điều chỉnh bảng luật

### Gỡ lỗi:
- Sử dụng script kiểm thử `test_fuzzy_pid.py` để xác minh chức năng
- Kiểm tra mức độ thuộc trong quá trình chạy
- Theo dõi giá trị hệ số để đảm bảo đúng mong đợi
- Đảm bảo bảng luật đầy đủ và nhất quán

## Kết luận

Bộ điều khiển FuzzyPIDController mang lại giải pháp điều khiển thích nghi, robust, xử lý tốt phi tuyến cho hệ bồn nước đôi. Cách tiếp cận dựa trên luật giúp dễ hiểu, dễ chỉnh sửa, phương pháp Mamdani đảm bảo điều khiển mượt mà. Triển khai tối ưu, tích hợp tốt với mô phỏng hiện có. 