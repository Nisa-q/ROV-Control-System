import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time

class DepthPID:
    def __init__(self, Kp=350.0, Ki=20.0, Kd=150.0, Ts=0.05, N=15.0, U_min=-400.0, U_max=400.0,alpha=0.4,deadband=0.01,max_rate=50.0):
        self.Kp, self.Ki, self.Kd = float(Kp), float(Ki), float(Kd)
        self.Ts, self.N = float(Ts), float(N)
        self.U_min, self.U_max = float(U_min), float(U_max)
        self.alpha = float(alpha)

        self.I = 0.0
        self.Df = 0.0
        self.prev_err = 0.0
        self.deadband=float(deadband)
        self.max_rate=float(max_rate)
        self.prev_meas=0.0#sensorun ılk halını hatırla
        self.prev_output=0.0#motorun eskı halını hatırla

        self.start_time=time.time()
        self.soft_start_duration=2.0
        #lowpassfilter için hafıza basta 0
        self.meas_filtered = 0.0
        self.first_run = True
        self.a = (2.0*self.N - self.Ts) / (2.0*self.N + self.Ts)
        self.b = (2.0*self.Kd*self.N) / (2.0*self.N + self.Ts)

    def step(self, set_depth_m: float, meas_depth_m: float):
        #acil durum freni
        if meas_depth_m>5.0 or meas_depth_m<-1.0:
            print(f"Acil durum:MANTIKSIZ VERİ:({meas_depth_m:.2f}m)")
            return 0.0,0.0,0.0,0.0,0.0,meas_depth_m

        #soft start için bır sey yapma ama olcmeye devam et ve 0.0 ı unut yavas yavas alphyala ugrasaya gerek yok 
        #suankı oldugun yerı baslnagıc noktası yap
        if (time.time()-self.start_time) <self.soft_start_duration:
            self.meas_filtered=meas_depth_m
            self.prev_meas=meas_depth_m
            return 0.0, 0.0, 0.0, 0.0, 0.0, meas_depth_m
        
        if self.first_run:
            self.meas_filtered = meas_depth_m # İlk veriyi olduğu gibi al
            self.first_run = False
        else:
            # Low Pass Filter Formülü:
            self.meas_filtered = (1 - self.alpha) * self.meas_filtered + self.alpha * meas_depth_m

        #deadband
        e = float(set_depth_m) - float(self.meas_filtered)
        if abs(e) < self.deadband:
            e = 0.0

        # 1. P Hesabı
        Up = self.Kp * e
        # 2. D Hesabı (Ölçüm Üzerinden Türev)
        meas_diff = self.meas_filtered - self.prev_meas
        derivative_input = -meas_diff
        self.Df = self.a * self.Df + self.b * derivative_input
        # 3. I Hesabı
        Ui_tmp = self.I + self.Ki * e * self.Ts
        # 4. Toplam
        U_total_tmp = Up + Ui_tmp + self.Df
        # 5. Limit
        U_clamped = max(self.U_min, min(self.U_max, U_total_tmp))
        # Anti-Windup
        if U_total_tmp == U_clamped:
            self.I = Ui_tmp
        # --- SLEW RATE LIMITER (Rampa) ---
        change = U_clamped - self.prev_output
        if change > self.max_rate:
            change = self.max_rate
        elif change < -self.max_rate:
            change = -self.max_rate
        U_final = self.prev_output + change
        # --- Hafıza Güncelleme ---
        self.prev_err = e
        self.prev_meas = self.meas_filtered
        self.prev_output = U_final 
        
        return e, Up, self.I, self.Df, U_final, self.meas_filtered

PWM_NEUTRAL = 1488
PWM_MIN, PWM_MAX = 1150, 1850
U_LIMIT_MIN = PWM_MIN - PWM_NEUTRAL
U_LIMIT_MAX = PWM_MAX - PWM_NEUTRAL

def plot_results(results_df,set_depth):
    plt.figure(figsize=(14,10))
    #1.derınlık grafigi
    plt.subplot(2,1,1)
    # 1. Ham Veri (Arka planda, açık mavi)
    plt.plot(results_df.index, results_df['Ölçülen_Derinlik'],
             label='Ham Sensör (Gürültülü)', color='lightblue', linewidth=1, linestyle='-')
    # 2. Filtreli Veri (Ön planda, koyu mavi veya turuncu)
    plt.plot(results_df.index, results_df['Filtreli_Derinlik'],
             label='Filtrelenmiş (PID Bunu Görüyor)', color='orange', linewidth=2.5) # Kalın çizgi
    plt.axhline(y=set_depth,color='red',linestyle='--',label=f'Hedef({set_depth}m)',linewidth=2)
    plt.title('Gürültü Filtresi Etkisi (Alpha=0.4)', fontsize=14)
    plt.ylabel('Derinlik (Metre)', fontsize=12)
    plt.legend()
    plt.grid(True, which='both', alpha=0.5)

    #2.modot cıkısı graıfıgı (pwm)
    plt.subplot(2, 1, 2)
    plt.plot(results_df.index, results_df['PWM'],
             label='Motor PWM', color='green', linewidth=1.5)
    #limitleri gorebılmek  ıcın ekledım        
    plt.axhline(y=PWM_NEUTRAL, color='gray', linestyle='-', label='Nötr (1488)', linewidth=2, alpha=0.7)
    plt.axhline(y=PWM_MAX, color='red', linestyle=':', label='Max Limit (1850)', linewidth=2)
    plt.axhline(y=PWM_MIN, color='red', linestyle=':', label='Min Limit (1150)', linewidth=2)
    plt.ylim(PWM_MIN - 50, PWM_MAX + 50)

    plt.axhline(y=PWM_NEUTRAL, color='gray', linestyle='-', label='Nötr', alpha=0.5)
    plt.title('Motor Tepkisi', fontsize=14)
    plt.ylabel('PWM (µs)', fontsize=12)
    plt.grid(True, which='both', alpha=0.5)

    plt.tight_layout()
    plt.show()

def run_pid_from_excel(excel_file_path: str, column_name: str, set_depth: float = 0.5):
    try:
        df = pd.read_excel(excel_file_path)
    except FileNotFoundError:
        print(f"HATA: '{excel_file_path}' dosyası bulunamadı.")
        return
    except Exception as e:
        print(f"HATA: Excel dosyası okunurken bir sorun oluştu: {e}")
        return
    if column_name not in df.columns:
        print(f"HATA: Excel dosyasında '{column_name}' adında bir sütun bulunamadı. Mevcut sütunlar: {list(df.columns)}")
        return
    pid = DepthPID(U_min=U_LIMIT_MIN, U_max=U_LIMIT_MAX,alpha=0.1)
    print(f"\nKLASİK PID + Anti-Windup (Excel Verisi ile). Hedef Derinlik: {set_depth:.3f} m\n")
    print("-" * 70)
    print(f"SIRA | Hedef | Ölçülen | Hata (e) | P | I | D | DeltaPWM | PWM")
    print("-" * 70)
    results = []
    for i, meas in enumerate(df[column_name]):
        # Eksik (NaN) veya geçersiz değerleri atla
        if pd.isna(meas) or not isinstance(meas, (int, float)):
            print(f"Uyarı: {i+1}. satırda geçersiz/eksik değer ('{meas}') atlanıyor.")
            continue

        e, P, I_val, D, U_clamped,meas_filt= pid.step(set_depth, float(meas))
        dpwm = U_clamped
        pwm = PWM_NEUTRAL + dpwm
        pwm = int(round(pwm))
        # PWM limit kontrolü (zaten U_clamped ile dpwm olarak kontrol edildi ama yine de emin olalım)
        pwm = max(PWM_MIN, min(PWM_MAX, pwm))
        print(
            f"{i+1:^4} | {set_depth:.3f} | {float(meas):.3f} | {e:+.3f} | "
            f"{P:+.1f} | {I_val:+.1f} | {D:+.1f} | {dpwm:+.1f} | {pwm} µs"
        )
        results.append({
            'Ölçülen_Derinlik': float(meas),
            'Filtreli_Derinlik': meas_filt,
            'Hata': e,
            'P': P,
            'I': I_val,
            'D': D,
            'DeltaPWM': dpwm,
            'PWM': pwm
        })
    print("-" * 70)
    print("Hesaplamalar Bitti.")
    results_df = pd.DataFrame(results)
    output_file_path = "pid_sonuclari.xlsx"
    try:
        results_df.to_excel(output_file_path, index=False)
        print(f"Sonuçlar '{output_file_path}' dosyasına kaydedildi.")
    except Exception as e:
        print(f"Uyarı: Sonuçlar Excel'e kaydedilemedi: {e}")

    print("Grafik çiziliyor...")
    plot_results(results_df, set_depth)

if __name__ == '__main__':
    excel_path = input("Excel dosyasının yolunu girin (örn. veriler.xlsx): ").strip()
    column_name = input("Ölçülen derinliklerin bulunduğu sütunun adını girin (örn. MeasDepth): ").strip()
    # Sabit hedef derinlik (0.5 m)
    HEDEF_DERINLIK = 0.5
    run_pid_from_excel(excel_path, column_name, HEDEF_DERINLIK)