# Kalman Filtresi – Detaylı Açıklama

Bu doküman, projedeki **temel lineer Kalman filtresini** (IMU verisi için) matematiksel modeli, uygulama içindeki konumu, nasıl çalıştığı ve kullanım senaryolarıyla birlikte açıklar.

---

## 0. Proje İçindeki Konum ve Uygulama

### 0.1 Kaynak Dosya

| Özellik | Değer |
|---------|-------|
| **Dosya** | `src/processing/kalman_variants.py` |
| **Sınıf** | `KalmanFilter` |
| **Temel sınıf** | `BaseEstimator` |

### 0.2 Hangi Uygulama?

Kalman filtresi, **İleri Düzey Kontrol, Stabilizasyon ve Otonom Seyrüsefer Sistemleri** projesinde kullanılır. Ana arayüz:

- **Streamlit web uygulaması** (`streamlit_app.py` → `src/ui/app.py`)
- **Çalıştırma:** `streamlit run streamlit_app.py`
- **Adres:** `http://localhost:8501`

### 0.3 Proje İçindeki Yerleşim

```
Kalman Fİltresi/
├── streamlit_app.py              # Giriş noktası
├── src/
│   ├── processing/
│   │   ├── kalman_variants.py    # KalmanFilter burada
│   │   ├── base.py               # Measurement, BaseEstimator
│   │   └── __init__.py           # KalmanFilter export
│   ├── core/
│   │   └── factories.py          # FilterFactory → "Kalman" → KalmanFilter
│   ├── ui/
│   │   └── app.py                # Streamlit UI, parametreler, CSV işleme
│   └── utils/
│       └── math_utils.py         # accel_to_roll_pitch
└── docs/
    └── kalman_filtresi_detay.md  # Bu dosya
```

---

## 1. Genel Bakış

Bu Kalman filtresi **IMU verisi** (jiroskop + ivmeölçer) için tasarlanmış, **lineer** bir filtre. İki modda çalışıyor:

- **Mod 1:** Ham ivmeölçer verisi kullanılır.
- **Mod 2:** Önceden hesaplanmış roll/pitch/yaw açıları kullanılır.

---

## 2. Durum Vektörü (State Vector)

### Mod 1: `use_attitude=False` (varsayılan)

```
x = [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z]ᵀ  (6 boyut)
```

- İlk 3 bileşen: jiroskop açısal hızları (rad/s)
- Son 3 bileşen: ivmeölçer değerleri (m/s²)

### Mod 2: `use_attitude=True`

```
x = [gyro_x, gyro_y, gyro_z, roll, pitch, yaw]ᵀ  (6 boyut)
```

- İlk 3 bileşen: jiroskop açısal hızları
- Son 3 bileşen: Euler açıları (radyan)

---

## 3. Parametreler

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `Q` | 0.01 | Süreç gürültü kovaryansı (skaler → 6×6 köşegen matris) |
| `R` | 0.1 | Ölçüm gürültü kovaryansı (skaler → 6×6 köşegen matris) |
| `P0` | 1.0 | Başlangıç durum kovaryansı (skaler → 6×6 köşegen matris) |
| `use_attitude` | False | Roll/pitch/yaw mı yoksa ham ivme mi kullanılacak |

---

## 4. Matematiksel Model

### 4.1 Durum Geçiş (State Transition)

Sabit hız modeli kullanılıyor; durum bir sonraki adımda aynı kalıyor:

```
x_{k|k-1} = F · x_{k-1}
```

`F` birim matris:

```
F = I₆ (6×6)
```

### 4.2 Ölçüm Modeli

Tüm durum bileşenleri doğrudan ölçülüyor:

```
z = H · x + v
```

`H` birim matris:

```
H = I₆ (6×6)
```

---

## 5. Algoritma Adımları

### 5.1 Tahmin (Prediction)

```
x_pred = F @ x = x
P_pred = F @ P @ Fᵀ + Q·I₆
```

### 5.2 Yenilik (Innovation)

```
y = z - H @ x_pred = z - x_pred
```

### 5.3 Yenilik Kovaryansı ve Kalman Kazancı

```
S = H @ P_pred @ Hᵀ + R·I₆ + 1e-6·I₆
K = P_pred @ Hᵀ @ S⁻¹
```

### 5.4 Güncelleme (Update)

```
x = x_pred + K @ y
P = (I₆ - K @ H) @ P_pred
```

---

## 6. Başlangıç (Initialization)

İlk ölçümde durum vektörü doğrudan ölçümden alınıyor. Roll/pitch verilmemişse ivmeölçerden hesaplanıyor:

```python
roll, pitch = accel_to_roll_pitch(accel_x, accel_y, accel_z)
yaw = 0.0
```

---

## 7. Girdi Formatı

`Measurement` nesnesi `data` sözlüğünde şunları içerir:

- `gyro`: `[gx, gy, gz]` (rad/s)
- `accel`: `[ax, ay, az]` (m/s²)
- Mod 2 için opsiyonel: `roll`, `pitch`, `yaw` (radyan)

---

## 8. Çıktı Formatı

### Mod 1 (`use_attitude=False`)

```python
{
    "state": [6 elemanlı liste],
    "covariance": [6×6 matris],
    "gyro_estimated": [gx, gy, gz],
    "accel_estimated": [ax, ay, az]
}
```

### Mod 2 (`use_attitude=True`)

```python
{
    "state": [6 elemanlı liste],
    "covariance": [6×6 matris],
    "gyro_estimated": [gx, gy, gz],
    "roll": float,
    "pitch": float,
    "yaw": float,
    "euler": [roll, pitch, yaw]
}
```

---

## 9. Hızlı Başvuru: Kalman Filtresini Çalıştırma

```bash
# 1. Proje dizinine git
cd "Kalman Fİltresi"

# 2. Sanal ortamı aktifleştir (varsa)
source venv/bin/activate  # Linux/Mac

# 3. Bağımlılıkları yükle
pip install -r requirements.txt

# 4. Streamlit uygulamasını başlat
streamlit run streamlit_app.py

# 5. Tarayıcıda http://localhost:8501 aç
# 6. CSV yükle → Kategori: Kalman Filtresi ve Türevleri → Algoritma: Kalman Filtresi
# 7. Q, R parametrelerini ayarla → Algoritmayı Uygula
```

---

## 10. Python Modülü Olarak Doğrudan Kullanım

```python
from src.core.factories import FilterFactory
from src.processing.base import Measurement

# Filtre oluştur
kf = FilterFactory.create("Kalman", Q=0.01, R=0.1)

# Ölçüm
m = Measurement(
    data={"gyro": [0.1, 0.2, 0.3], "accel": [0.0, 0.0, 9.81]},
    meta={"timestamp": 0.0}
)

# Güncelle
result = kf.update(m, dt=0.01)
# result: {"state": [...], "gyro_estimated": [...], "accel_estimated": [...]}
```
