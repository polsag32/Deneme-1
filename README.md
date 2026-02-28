# Kalman Filtresi - IMU Veri İşleme

İleri Düzey Kontrol, Stabilizasyon ve Otonom Seyrüsefer Sistemleri projesi için IMU (jiroskop + ivmeölçer) verisi işleme uygulaması.

## Kurulum

```bash
python3 -m venv venv
source venv/bin/activate   # Linux/Mac
# veya: venv\Scripts\activate   # Windows
pip install -r requirements.txt
```

## Çalıştırma

```bash
streamlit run streamlit_app.py
```

Tarayıcıda `http://localhost:8501` adresini açın.

## Kullanım

1. CSV dosyası yükleyin (gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z sütunları)
2. Sütun eşlemesini yapın
3. Q, R parametrelerini ayarlayın
4. "Algoritmayı Uygula" butonuna basın

## Proje Yapısı

- `src/processing/kalman_variants.py` - KalmanFilter sınıfı
- `src/core/factories.py` - FilterFactory
- `src/ui/app.py` - Streamlit arayüzü
- `docs/kalman_filtresi_detay.md` - Detaylı dokümantasyon
