# IoT Gas/Butane Monitoring System with MQ-2 and Humidity Compensation

## 1. Giới thiệu

Đây là project xây dựng hệ thống IoT giám sát khí gas/butan sử dụng cảm biến MQ-2, cảm biến nhiệt độ - độ ẩm DHT22 và vi điều khiển ESP8266. Mục tiêu chính của hệ thống là phát hiện sự xuất hiện của khí gas/butan trong môi trường thực tế, đồng thời giảm sai lệch do độ ẩm gây ra bằng mô hình bù độ ẩm.

Trong môi trường nóng ẩm, đặc biệt là điều kiện thực tế tại Việt Nam, cảm biến khí MQ-2 có thể bị ảnh hưởng bởi hơi nước trong không khí. Điều này làm thay đổi điện trở cảm biến và khiến giá trị ppm tính toán bị phóng đại nếu chỉ sử dụng mô hình không bù. Vì vậy, hệ thống sử dụng DHT22 để đo nhiệt độ và độ ẩm tương đối, sau đó tính độ ẩm tuyệt đối `AH` và đưa vào mô hình bù trước khi ước lượng nồng độ khí.

Project này phù hợp với môn học **IoT và ứng dụng**, tập trung vào các nội dung:

* Đọc dữ liệu cảm biến khí MQ-2.
* Đọc nhiệt độ và độ ẩm từ DHT22.
* Tính toán điện trở cảm biến `Rs`.
* Tính ppm theo mô hình không bù.
* Tính ppm theo mô hình có bù ảnh hưởng độ ẩm.
* So sánh đáp ứng giữa hai mô hình.
* Cảnh báo khí gas bằng LED và buzzer.
* Ghi log dữ liệu phục vụ vẽ đồ thị và phân tích.

## 2. Tính năng chính

* Đọc tín hiệu analog từ cảm biến MQ-2.
* Đo nhiệt độ và độ ẩm bằng DHT22.
* Tính độ ẩm tuyệt đối `AH`.
* Giới hạn `AH` trong vùng mô hình để tránh ngoại suy quá xa.
* Tính nồng độ khí theo hai nhánh:

  * `ppm_nocomp`: mô hình không bù độ ẩm.
  * `ppm_comp`: mô hình có bù độ ẩm.
* Làm mượt giá trị `Rs` bằng trung bình trượt.
* Ghi log dữ liệu qua Serial Monitor.
* Cảnh báo khi phát hiện khí gas:

  * LED xanh báo trạng thái bình thường.
  * LED đỏ báo trạng thái cảnh báo.
  * Buzzer phát âm cảnh báo.

## 3. Phần cứng sử dụng

| Thành phần                | Chức năng                  |
| ------------------------- | -------------------------- |
| ESP8266 ESP-12E / NodeMCU | Vi điều khiển trung tâm    |
| MQ-2                      | Cảm biến khí gas/butan     |
| DHT22                     | Cảm biến nhiệt độ và độ ẩm |
| LED xanh                  | Báo trạng thái bình thường |
| LED đỏ                    | Báo trạng thái cảnh báo    |
| Buzzer                    | Cảnh báo âm thanh          |
| Breadboard, dây nối       | Lắp mạch thử nghiệm        |
| Nguồn 5V USB/Adapter      | Cấp nguồn cho hệ thống     |

## 4. Sơ đồ kết nối

| Thiết bị   | Chân ESP8266 | Mô tả                               |
| ---------- | ------------ | ----------------------------------- |
| MQ-2 AO    | A0           | Đọc tín hiệu analog từ cảm biến khí |
| DHT22 DATA | D3           | Đọc nhiệt độ và độ ẩm               |
| LED đỏ     | D7           | Cảnh báo khi phát hiện khí gas      |
| LED xanh   | D8           | Báo trạng thái bình thường          |
| Buzzer     | D0           | Cảnh báo âm thanh                   |
| VCC MQ-2   | 5V           | Cấp nguồn cảm biến MQ-2             |
| VCC DHT22  | 3.3V hoặc 5V | Cấp nguồn DHT22                     |
| GND        | GND          | Mass chung                          |

> Lưu ý: Nếu dùng NodeMCU/ESP8266, chân A0 chỉ chịu được điện áp trong giới hạn của board. Trong project này, module MQ-2 đã có mạch chia áp phù hợp nên có thể đọc trực tiếp qua A0.

## 5. Nguyên lý hoạt động

Hệ thống hoạt động theo các bước sau:

1. ESP8266 đọc giá trị ADC từ cảm biến MQ-2.
2. Giá trị ADC được chuyển đổi sang điện áp `Vout`.
3. Từ `Vout`, hệ thống tính điện trở cảm biến `Rs`.
4. DHT22 đo nhiệt độ `T` và độ ẩm tương đối `RH`.
5. Từ `T` và `RH`, hệ thống tính độ ẩm tuyệt đối `AH`.
6. Hệ thống tính song song hai giá trị ppm:

   * ppm không bù: chỉ dùng `Rs/Ro`.
   * ppm có bù: dùng `AH` để hiệu chỉnh ảnh hưởng độ ẩm.
7. Giá trị ppm có bù được dùng cho cảnh báo chính.
8. Dữ liệu được in ra Serial Monitor để lưu log và vẽ đồ thị.

## 6. Công thức tính toán

### 6.1. Chuyển đổi ADC sang điện áp

```cpp
Vout = ADC * (3.3 / 1023.0)
```

### 6.2. Tính điện trở cảm biến MQ-2

```cpp
Rs = RL * ((VCC - Vout) / Vout)
```

Trong đó:

* `Rs`: điện trở cảm biến MQ-2 tại thời điểm đo.
* `RL`: điện trở tải của module MQ-2.
* `VCC`: điện áp cấp cho cảm biến.
* `Vout`: điện áp đầu ra đọc được từ chân analog.

### 6.3. Tính độ ẩm tuyệt đối AH

```cpp
AH = (RH * 6.11 * pow(10, (7.5 * T) / (T + 237.3))) / 1013.25
```

Trong đó:

* `T`: nhiệt độ môi trường, đơn vị °C.
* `RH`: độ ẩm tương đối, đơn vị %.
* `AH`: độ ẩm tuyệt đối dùng cho mô hình bù.

### 6.4. Mô hình bù ảnh hưởng độ ẩm

```cpp
Re = a * exp(-b * AH) + c
```

Trong project này, các hệ số được tuning thực nghiệm:

```cpp
a = 4.54
b = 1.20
c = 0.60
```

Sau đó tính điện trở đã bù:

```cpp
Ros = Rs / Re
```

### 6.5. Tính ppm có bù

```cpp
ppm_comp = k * exp(-lambda * (Ros / Ro)) + m
```

Các hệ số tuning thực nghiệm:

```cpp
k = 1120.68
lambda = 0.97
m = 1.99
```

### 6.6. Tính ppm không bù

```cpp
ppm_nocomp = A * pow(Rs / Ro, B)
```

Các hệ số tuning thực nghiệm cho khí gas/butan:

```cpp
A = 1023.0
B = -2.102
```

Mô hình không bù không sử dụng nhiệt độ, độ ẩm, `AH`, `Re` hay `Ros`. Nhánh này được dùng để so sánh ảnh hưởng của độ ẩm lên kết quả tính ppm.

## 7. Ý nghĩa của mô hình bù độ ẩm

Cảm biến MQ-2 là cảm biến bán dẫn oxit kim loại, có thể bị ảnh hưởng bởi hơi nước trong không khí. Khi độ ẩm thay đổi, điện trở cảm biến có thể thay đổi ngay cả khi lượng khí gas không thay đổi đáng kể. Điều này làm cho mô hình không bù có thể tính ra giá trị ppm cao hơn thực tế.

Mô hình bù độ ẩm giúp:

* Giảm sai lệch ppm do độ ẩm môi trường.
* Làm kết quả đo ổn định hơn.
* Giảm khả năng cảnh báo sai.
* Phù hợp hơn với môi trường thực tế có độ ẩm cao.
* Cải thiện độ tin cậy của hệ thống cảnh báo khí gas.

Hệ thống không nhằm định danh chính xác khí LPG/butan, vì MQ-2 có thể phản ứng với nhiều khí cháy khác nhau. Mục tiêu của project là giám sát tương đối sự xuất hiện của khí gas/butan và đánh giá ảnh hưởng của độ ẩm lên cảm biến MQ-2.

## 8. Cài đặt phần mềm

### 8.1. Yêu cầu

* Arduino IDE
* Board ESP8266 đã được cài trong Arduino IDE
* Thư viện DHT sensor library
* Thư viện Adafruit Unified Sensor

### 8.2. Cài board ESP8266

Trong Arduino IDE:

1. Vào `File` → `Preferences`.
2. Thêm URL sau vào `Additional Boards Manager URLs`:

```text
http://arduino.esp8266.com/stable/package_esp8266com_index.json
```

3. Vào `Tools` → `Board` → `Boards Manager`.
4. Tìm `ESP8266` và cài đặt.

### 8.3. Cài thư viện

Vào `Tools` → `Manage Libraries`, tìm và cài:

```text
DHT sensor library
Adafruit Unified Sensor
```

## 9. Cách chạy project

1. Kết nối phần cứng theo sơ đồ.
2. Mở file code Arduino `.ino`.
3. Chọn board ESP8266 phù hợp, ví dụ:

```text
NodeMCU 1.0 (ESP-12E Module)
```

4. Chọn đúng cổng COM.
5. Upload code lên ESP8266.
6. Mở Serial Monitor với baudrate:

```text
115200
```

7. Quan sát log dữ liệu gồm:

```text
ADC
Vout
Temperature
Humidity
AH_raw
AH_used
Rs
Re
Ros
ppm_comp
ppm_nocomp
alarm_state
```

## 10. Kết quả thực nghiệm

Trong thí nghiệm, khí gas/butan từ bật lửa được đưa vào buồng đo. Hệ thống ghi nhận sự thay đổi của cảm biến MQ-2 và tính toán hai giá trị ppm:

* `ppm_nocomp`: ppm không bù độ ẩm.
* `ppm_comp`: ppm có bù độ ẩm.

Kết quả cho thấy mô hình không bù có thể tăng vọt khi điều kiện môi trường thay đổi, trong khi mô hình có bù cho giá trị ổn định hơn và gần với mức khí thử nghiệm hơn.

Quy ước đồ thị:

* Đường xanh nước biển: ppm không bù.
* Đường đỏ: ppm có bù.
* Đường xanh lá hoặc đen: mức khí bơm vào buồng.
* Trục Y trái: nồng độ ppm.
* Trục Y phải: giá trị độ ẩm tuyệt đối AH, nếu có hiển thị.

## 11. Cảnh báo và giới hạn

Project này chỉ phục vụ mục đích học tập, nghiên cứu và thử nghiệm trong môn IoT.

Một số giới hạn:

* MQ-2 không phải cảm biến chuyên dụng để định danh chính xác LPG/butan.
* MQ-2 có thể phản ứng với nhiều loại khí khác nhau như LPG, methane, propane, hydrogen, alcohol và khói.
* Lượng khí bơm từ bật lửa chỉ là mức thử nghiệm tương đối, không phải nồng độ chuẩn tuyệt đối.
* Các hệ số tuning chỉ phù hợp với hệ thống phần cứng và điều kiện thử nghiệm hiện tại.
* Không nên sử dụng project này thay thế cho thiết bị cảnh báo gas thương mại trong các môi trường yêu cầu an toàn cao.

## 12. Hướng phát triển

Trong tương lai, hệ thống có thể được mở rộng thêm:

* Gửi dữ liệu lên Blynk, Firebase, ThingsBoard hoặc MQTT server.
* Hiển thị biểu đồ ppm và AH theo thời gian thực.
* Lưu lịch sử cảnh báo.
* Gửi thông báo đến điện thoại khi phát hiện khí gas.
* Hiệu chuẩn bằng khí chuẩn để cải thiện độ chính xác.
* Thử nghiệm thêm trong nhiều điều kiện nhiệt độ và độ ẩm khác nhau.
* Mở rộng mô hình bù cho nhiều vùng AH hơn.
* Thiết kế PCB để hệ thống gọn và ổn định hơn.

## 13. Cấu trúc repo đề xuất

```text
.
├── code/
│   └── mq2_dht22_humidity_compensation.ino
├── data/
│   └── experimental_log.txt
├── figures/
│   ├── block_diagram.png
│   ├── real_system.png
│   └── result_plot.png
├── report/
│   └── IoT_report.pdf
└── README.md
```

## 14. Tác giả

Sinh viên thực hiện:

```text
Lê Đức Bình
```

## 15. Tài liệu tham khảo

* Hanwei Electronics, MQ-2 Semiconductor Sensor for Combustible Gas Datasheet.
* Aosong Electronics, DHT22/AM2302 Digital Temperature and Humidity Sensor Datasheet.
* Espressif Systems, ESP8266EX Datasheet.
* M. Yan et al., "Humidity compensation based on power-law response for MOS sensors to VOCs", Sensors and Actuators B: Chemical, 2021.
