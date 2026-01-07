import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2 

# ================== AYARLAR ==================
PWM_NORMAL = 50 
PWM_SLOW = 40

FRAME_W, FRAME_H = 640, 480
ROI_TOP = 220
THRESH = 70  

CENTER_DEADBAND = 60

# ================== PID AYARLARI ==================
KP = 0.004 
KI = 0.000
KD = 0.6    
MAX_CORR = 7 

# ================== PWM RAMPA AYARLARI ==================
RAMP_LIMIT = 5  

# ================== VİRAJ AYARLARI (YENİ!) ==================
SHARP_TURN_ERROR = 60    
EXTREME_TURN_ERROR = 100  

# Keskin viraj motor hızları (L dönüşleri için)
SHARP_INNER_SPEED = 0.3    # İç tekerlek yavaş
SHARP_OUTER_SPEED = 1.4    # Dış tekerlek hızlı

# Çok keskin viraj
EXTREME_INNER_SPEED = 0.15  # İç tekerlek çok yavaş
EXTREME_OUTER_SPEED = 1.7   # Dış tekerlek çok hızlı

STOP_CM = 10.0
SLOW_MIN = 10.0
SLOW_MAX = 20.0

DEBUG = True

# ================== PINLER ==================
AIN1, AIN2, PWMA = 23, 24, 18
BIN1, BIN2, PWMB = 5, 6, 13
STBY = 25
FRONT_TRIG, FRONT_ECHO = 17, 21

# ================== GPIO ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [AIN1, AIN2, BIN1, BIN2, PWMA, PWMB, STBY, FRONT_TRIG]:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(FRONT_ECHO, GPIO.IN)

pwmA = GPIO.PWM(PWMA, 1000)
pwmB = GPIO.PWM(PWMB, 1000)
pwmA.start(0)
pwmB.start(0)
GPIO.output(STBY, GPIO.HIGH)

# Global rampa değişkenleri
current_pwm_A = 0
current_pwm_B = 0

# ================== MOTOR KONTROL ==================
def set_motors(target_left, target_right, reverse_left=False, reverse_right=False):
    global current_pwm_A, current_pwm_B
    
    target_left = max(0, min(100, target_left))
    target_right = max(0, min(100, target_right))

    # Rampa kontrolü
    if target_left > current_pwm_A:
        current_pwm_A = min(target_left, current_pwm_A + RAMP_LIMIT)
    elif target_left < current_pwm_A:
        current_pwm_A = max(target_left, current_pwm_A - RAMP_LIMIT)
    
    if target_right > current_pwm_B:
        current_pwm_B = min(target_right, current_pwm_B + RAMP_LIMIT)
    elif target_right < current_pwm_B:
        current_pwm_B = max(target_right, current_pwm_B - RAMP_LIMIT)

    # Sol motor yön
    if reverse_left:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
    else:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)

    # Sağ motor yön (ters bağlantı)
    if reverse_right:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
    else:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)

    pwmA.ChangeDutyCycle(current_pwm_A)
    pwmB.ChangeDutyCycle(current_pwm_B)

def stop():
    global current_pwm_A, current_pwm_B
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    current_pwm_A = 0
    current_pwm_B = 0

# ================== KAMERA ==================
try:
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
except Exception as e:
    print(f"Kamera hatası: {e}")
    exit()

# ================== ÇİZGİ BUL ==================
def find_line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi = gray[ROI_TOP:, :]
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    
    _, binary = cv2.threshold(blur, THRESH, 255, cv2.THRESH_BINARY_INV)
    
    # Morfolojik işlemler
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.erode(binary, kernel, iterations=1)
    binary = cv2.dilate(binary, kernel, iterations=2)

    M = cv2.moments(binary)
    mid = binary.shape[1] // 2

    if M["m00"] > 1000:
        cx = int(M["m10"] / M["m00"])
        return cx, mid, binary
    return None, mid, binary

# ================== MESAFE ==================
def measure_distance():
    GPIO.output(FRONT_TRIG, False)
    time.sleep(0.0001)
    GPIO.output(FRONT_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(FRONT_TRIG, False)

    timeout = time.time() + 0.05
    pulse_start = time.time()
    while GPIO.input(FRONT_ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    pulse_end = time.time()
    timeout = time.time() + 0.05
    while GPIO.input(FRONT_ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    dist = (pulse_end - pulse_start) * 17150
    return round(dist, 2) if 2 < dist < 300 else None

# ================== ANA ==================
def main():

    lost_frames = 0
    last_error = 0
    integral = 0
    previous_error = 0

    try:
        while True:
            frame = picam2.capture_array()
            dist = measure_distance()

            # Engel kontrolü
            if dist is not None and dist < STOP_CM:
                stop()
                integral = 0
                previous_error = 0
                continue

            target_speed = PWM_SLOW if (dist and SLOW_MIN <= dist <= SLOW_MAX) else PWM_NORMAL
            cx, mid, binary = find_line(frame)

            # -------- ÇİZGİ KAYIP --------
            if cx is None:
                lost_frames += 1
                integral = 0
                previous_error = 0
                
                if lost_frames > 20:
                    stop()
                else:
                    # Son hataya göre dön
                    if last_error > 0:
                        set_motors(target_speed, target_speed, 
                                 reverse_left=False, reverse_right=True)
                    else:
                        set_motors(target_speed, target_speed,
                                 reverse_left=True, reverse_right=False)
                continue

            # -------- ÇİZGİ BULUNDU --------
            lost_frames = 0
            error = cx - mid
            abs_error = abs(error)
            
            # Ölü bölge
            if abs_error < CENTER_DEADBAND:
                error = 0
                integral = 0
                previous_error = 0

            # PID hesaplama
            P = error
            integral = integral + error
            integral = max(min(integral, 100), -100)
            
            D = error - previous_error
            D = 0.8 * D
            
            pid_value = (KP * P) + (KI * integral) + (KD * D)
            previous_error = error
            last_error = error

            correction = int(pid_value)
            correction = max(-MAX_CORR, min(MAX_CORR, correction))
            
            # Küçük düzeltmeleri yumuşat
            if abs(correction) < 3:
                correction = 0

            if abs_error > EXTREME_TURN_ERROR:
                # ÇOK KESKİN VİRAJ
                if error > 0:  # Sağa dön
                    left_speed = int(target_speed * EXTREME_OUTER_SPEED)
                    right_speed = int(target_speed * EXTREME_INNER_SPEED)
                else:  # Sola dön
                    left_speed = int(target_speed * EXTREME_INNER_SPEED)
                    right_speed = int(target_speed * EXTREME_OUTER_SPEED)
                turn_mode = "🔴 EXTREME"
                
            elif abs_error > SHARP_TURN_ERROR:
                # KESKİN VİRAJ (L)
                if error > 0:  # Sağa dön
                    left_speed = int(target_speed * SHARP_OUTER_SPEED)
                    right_speed = int(target_speed * SHARP_INNER_SPEED)
                else:  # Sola dön
                    left_speed = int(target_speed * SHARP_INNER_SPEED)
                    right_speed = int(target_speed * SHARP_OUTER_SPEED)
                turn_mode = "🟠 SHARP"
                
            else:
                # NORMAL PID
                left_speed = target_speed - correction
                right_speed = target_speed + correction
                turn_mode = "🟢 PID"
            
            set_motors(left_speed, right_speed)

            # -------- DEBUG --------
            if DEBUG:
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.line(dbg, (mid, 0), (mid, dbg.shape[0]), (0, 255, 0), 2)
                if cx:
                    cv2.circle(dbg, (cx, dbg.shape[0]//2), 8, (0, 0, 255), -1)
                
                info_text = f"E:{error:+4d} | {turn_mode} | L:{left_speed} R:{right_speed}"
                cv2.putText(dbg, info_text, (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow("Line Follower", dbg)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu")

    finally:
        stop()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("Program kapatıldı")

if __name__ == "__main__":
    main()
