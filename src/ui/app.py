"""Streamlit UI for IMU filter application."""

from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

# Proje kökünü Python path'e ekle
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.core.factories import FilterFactory
from src.processing.base import Measurement

_TIME_COL_NAMES = ["time_s", "time", "timestamp", "t"]


def _detect_imu_columns(df: pd.DataFrame) -> dict[str, dict[str, str]]:
    """
    Sütunları tara, sonu 1 veya 2 ile biten IMU sütunlarını tespit et.
    Dönüş: {"1": {"accel_x": "AcX1", ...}, "2": {...}}
    """
    result: dict[str, dict[str, str]] = {}
    cols = [str(c) for c in df.columns]
    for suffix in ("1", "2"):
        mapping: dict[str, str] = {}
        for col in cols:
            if len(col) < 2 or not col[-1] == suffix:
                continue
            base = col[:-1].upper()
            if base == "ACX":
                mapping["accel_x"] = col
            elif base == "ACY":
                mapping["accel_y"] = col
            elif base == "ACZ":
                mapping["accel_z"] = col
            elif base == "GYX":
                mapping["gyro_x"] = col
            elif base == "GYY":
                mapping["gyro_y"] = col
            elif base == "GYZ":
                mapping["gyro_z"] = col
        if len(mapping) == 6:
            result[suffix] = mapping
    return result


def _detect_time_column(df: pd.DataFrame) -> str | None:
    """Zaman sütununu tespit et (time_s, time, timestamp, t)."""
    cols_lower = {str(c).lower(): c for c in df.columns}
    for name in _TIME_COL_NAMES:
        if name in cols_lower:
            return cols_lower[name]
    return None


def _build_column_mapping(
    df: pd.DataFrame, imu_choice: str, imu_available: dict[str, dict[str, str]]
) -> dict[str, str]:
    """Seçilen IMU için column_mapping oluştur."""
    suffix = "1" if imu_choice == "IMU1" else "2"
    mapping = dict(imu_available.get(suffix, {}))
    time_col = _detect_time_column(df)
    mapping["time_s"] = time_col
    return mapping


def _row_to_measurement(row: pd.Series, column_mapping: dict[str, str]) -> Measurement:
    """CSV satırından Measurement oluşturur."""
    gyro = [
        float(row[column_mapping["gyro_x"]]),
        float(row[column_mapping["gyro_y"]]),
        float(row[column_mapping["gyro_z"]]),
    ]
    accel = [
        float(row[column_mapping["accel_x"]]),
        float(row[column_mapping["accel_y"]]),
        float(row[column_mapping["accel_z"]]),
    ]
    time_col = column_mapping.get("time_s")
    if time_col and time_col in row.index:
        timestamp = float(row[time_col])
    else:
        timestamp = float(row.name) if row.name is not None else 0.0
    return Measurement(
        data={"gyro": gyro, "accel": accel},
        meta={"timestamp": timestamp, "row_index": row.name},
    )


def _run_filter_on_dataframe(
    df: pd.DataFrame,
    filter_name: str,
    params: dict,
    column_mapping: dict[str, str],
    dt: float,
) -> list[dict]:
    """DataFrame üzerinde tek filtre çalıştırır."""
    results = []
    filt = FilterFactory.create(filter_name, **params)
    for _, row in df.iterrows():
        m = _row_to_measurement(row, column_mapping)
        out = filt.update(m, dt)
        out = out.copy()
        out["timestamp"] = m.meta.get("timestamp", 0.0)
        out["original_gyro"] = m.data["gyro"]
        out["original_accel"] = m.data["accel"]
        results.append(out)
    return results


def _convert_results_to_dataframe(results: list[dict]) -> pd.DataFrame:
    """Filtre çıktılarını DataFrame'e dönüştürür."""
    rows = []
    for r in results:
        row = {
            "timestamp": r.get("timestamp", 0),
            "original_gyro_x": r["original_gyro"][0],
            "original_gyro_y": r["original_gyro"][1],
            "original_gyro_z": r["original_gyro"][2],
            "original_accel_x": r["original_accel"][0],
            "original_accel_y": r["original_accel"][1],
            "original_accel_z": r["original_accel"][2],
        }
        if "gyro_estimated" in r:
            row["filtered_gyro_x"] = r["gyro_estimated"][0]
            row["filtered_gyro_y"] = r["gyro_estimated"][1]
            row["filtered_gyro_z"] = r["gyro_estimated"][2]
        if "accel_estimated" in r:
            row["filtered_accel_x"] = r["accel_estimated"][0]
            row["filtered_accel_y"] = r["accel_estimated"][1]
            row["filtered_accel_z"] = r["accel_estimated"][2]
        if "roll" in r:
            row["roll_filtered"] = r["roll"]
            row["pitch_filtered"] = r["pitch"]
            row["yaw_filtered"] = r["yaw"]
        state = r.get("state", [])
        for i, v in enumerate(state):
            row[f"state_{i}"] = v
        rows.append(row)
    return pd.DataFrame(rows)


def _variance_score(results: list[dict]) -> float:
    """Roll/pitch/yaw veya state varyansları toplamı (düşük = daha iyi)."""
    if not results:
        return float("inf")
    import numpy as np
    rolls = [r.get("roll", 0) for r in results if "roll" in r]
    pitches = [r.get("pitch", 0) for r in results if "pitch" in r]
    yaws = [r.get("yaw", 0) for r in results if "yaw" in r]
    if rolls or pitches or yaws:
        return float(np.var(rolls) + np.var(pitches) + np.var(yaws))
    # Mod 1: gyro + accel (tüm 6 state bileşeni)
    state_vars = [np.var([r["state"][i] for r in results]) for i in range(6)]
    return float(sum(state_vars))


def run_app() -> None:
    """Streamlit uygulamasını çalıştırır."""
    st.set_page_config(page_title="Kalman Filtresi - IMU İşleme", layout="wide")
    st.title("İleri Düzey Kontrol, Stabilizasyon ve Otonom Seyrüsefer Sistemleri")
    st.subheader("Kalman Filtresi ve Türevleri")

    # Sidebar - Algoritma ve parametreler
    with st.sidebar:
        st.header("Algoritma")
        category = st.selectbox(
            "Kategori",
            ["Kalman Filtresi ve Türevleri"],
            index=0,
        )
        algorithms = ["Kalman Filtresi"]
        algo_idx = st.selectbox(
            "Algoritma",
            algorithms,
            index=0,
        )
        filter_name = "Kalman"

        st.header("Parametreler")
        Q = st.number_input("Q (Süreç Gürültü)", value=0.01, min_value=0.0001, max_value=1.0, step=0.001, format="%.4f")
        R = st.number_input("R (Ölçüm Gürültü)", value=0.1, min_value=0.0001, max_value=1.0, step=0.01, format="%.4f")
        P0 = st.number_input("P0 (Başlangıç Kovaryans)", value=1.0, min_value=0.01, max_value=10.0, step=0.1)
        use_attitude = st.checkbox("Roll/Pitch/Yaw modu (use_attitude)", value=False)
        dt = st.number_input("Zaman adımı (dt) [s]", value=0.01, min_value=0.001, step=0.001, format="%.3f")

    # Ana alan - CSV yükleme
    uploaded = st.file_uploader("CSV dosyası yükle", type=["csv"])
    if not uploaded:
        st.info("CSV dosyası yükleyerek başlayın.")
        return

    # CSV okuma ayarları (genişletilebilir panel)
    with st.expander("CSV Okuma Ayarları", expanded=False):
        encoding = st.selectbox(
            "Karakter kodlaması",
            ["utf-8", "utf-8-sig", "cp1254", "iso-8859-9", "latin-1"],
            index=0,
            help="utf-8-sig: BOM'lu UTF-8, cp1254: Windows Türkçe",
        )
        delimiter = st.selectbox(
            "Sütun ayırıcı",
            [",", ";", "\t", "|"],
            format_func=lambda x: {",": "Virgül (,)", ";": "Noktalı virgül (;)", "\t": "Sekme (Tab)", "|": "Dikey çizgi (|)"}[x],
            index=0,
        )
        decimal_char = st.selectbox(
            "Ondalık ayırıcı",
            [".", ","],
            format_func=lambda x: "Nokta (.)" if x == "." else "Virgül (,)",
            index=0,
            help="Sayılar 1.5 mi yoksa 1,5 mi yazılmış?",
        )

    # CSV'yi oku (hata durumunda alternatif encoding dene)
    try:
        df = pd.read_csv(
            uploaded,
            encoding=encoding,
            sep=delimiter,
            decimal=decimal_char,
            skipinitialspace=True,
        )
    except UnicodeDecodeError:
        # Encoding başarısız olursa latin-1 ile dene (her byte geçerli)
        uploaded.seek(0)
        df = pd.read_csv(uploaded, encoding="latin-1", sep=delimiter, decimal=decimal_char, skipinitialspace=True)
        st.warning(f"'{encoding}' ile okunamadı, latin-1 kullanıldı.")

    # Sütun adlarındaki BOM ve boşlukları temizle
    df.columns = df.columns.str.strip().str.replace("\ufeff", "", regex=False)

    st.write("Önizleme:", df.head())
    if df.empty:
        st.error("CSV boş görünüyor. 'CSV Okuma Ayarları'ndan kodlama ve ayırıcıyı kontrol edin.")
        return

    # Sütun eşleme - IMU1/IMU2 otomatik tespit
    st.subheader("Sütun Eşleme")
    imu_available = _detect_imu_columns(df)
    if not imu_available:
        st.error(
            "CSV'de IMU1/IMU2 formatı bulunamadı. Sütunlar AcX1, AcY1, AcZ1, GyX1, GyY1, GyZ1 "
            "(veya 2 ile bitenler) olmalıdır."
        )
        return

    imu_options = [f"IMU{k}" for k in sorted(imu_available.keys())]
    imu_choice = st.selectbox("IMU Seçin", imu_options, index=0)
    column_mapping = _build_column_mapping(df, imu_choice, imu_available)
    mapping_lines = [
        f"- **gyro_x:** {column_mapping.get('gyro_x', '—')}",
        f"- **gyro_y:** {column_mapping.get('gyro_y', '—')}",
        f"- **gyro_z:** {column_mapping.get('gyro_z', '—')}",
        f"- **accel_x:** {column_mapping.get('accel_x', '—')}",
        f"- **accel_y:** {column_mapping.get('accel_y', '—')}",
        f"- **accel_z:** {column_mapping.get('accel_z', '—')}",
        f"- **time_s:** {column_mapping.get('time_s') or '(otomatik bulunamadı)'}",
    ]
    st.info(f"**Seçilen: {imu_choice}**\n\n" + "\n".join(mapping_lines))

    if st.button("Algoritmayı Uygula"):
        params = {"Q": Q, "R": R, "P0": P0, "use_attitude": use_attitude}
        results = _run_filter_on_dataframe(df, filter_name, params, column_mapping, dt)
        out_df = _convert_results_to_dataframe(results)
        st.subheader("Sonuçlar")
        st.dataframe(out_df, use_container_width=True)

        # Grafikler
        st.subheader("Grafikler")
        t = out_df["timestamp"].values
        for axis, label in [("x", "X"), ("y", "Y"), ("z", "Z")]:
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=t, y=out_df[f"original_gyro_{axis}"], name=f"Ham gyro_{axis}", mode="lines"))
            fig.add_trace(go.Scatter(x=t, y=out_df[f"filtered_gyro_{axis}"], name=f"Filtrelenmiş gyro_{axis}", mode="lines"))
            fig.update_layout(title=f"Gyro {label}", xaxis_title="Zaman", height=300)
            st.plotly_chart(fig, use_container_width=True)
        for axis, label in [("x", "X"), ("y", "Y"), ("z", "Z")]:
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=t, y=out_df[f"original_accel_{axis}"], name=f"Ham accel_{axis}", mode="lines"))
            fig.add_trace(go.Scatter(x=t, y=out_df[f"filtered_accel_{axis}"], name=f"Filtrelenmiş accel_{axis}", mode="lines"))
            fig.update_layout(title=f"Accel {label}", xaxis_title="Zaman", height=300)
            st.plotly_chart(fig, use_container_width=True)

        # CSV indirme
        csv_bytes = out_df.to_csv(index=False).encode("utf-8")
        st.download_button("Sonuçları CSV olarak indir", csv_bytes, "filtered_results.csv", "text/csv")

    # Otomatik parametre arama
    st.subheader("Otomatik Parametre Arama")
    if st.button("Otomatik parametre ara"):
        import numpy as np
        results_list = []
        best_score = float("inf")
        best_Q, best_R = Q, R
        for q in np.linspace(0.0001, 1.0, 15):
            for r in np.linspace(0.0001, 1.0, 15):
                params = {"Q": q, "R": r, "P0": P0, "use_attitude": use_attitude}
                res = _run_filter_on_dataframe(df, filter_name, params, column_mapping, dt)
                sc = _variance_score(res)
                results_list.append({"Q": q, "R": r, "skor": sc})
                if sc < best_score:
                    best_score = sc
                    best_Q, best_R = q, r

        search_df = pd.DataFrame(results_list).sort_values("skor")
        st.success(f"Önerilen: Q={best_Q:.4f}, R={best_R:.4f} (skor={best_score:.6f})")
        st.dataframe(search_df, use_container_width=True, height=300)
        param_csv = search_df.to_csv(index=False).encode("utf-8")
        st.download_button(
            "Parametre arama sonuçlarını CSV olarak indir",
            param_csv,
            "parametre_arama_sonuclari.csv",
            "text/csv",
            key="param_search_download",
        )
