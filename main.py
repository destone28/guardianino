from secrets import WIFI_SSID, WIFI_PASS, DEVICE_ID, SECRET_KEY
from arduino_iot_cloud import Task, ArduinoCloudClient, async_wifi_connection
import network
import time
import os
import pyb
import sensor
import image
import audio
import logging
from ulab import numpy as np

# Configurazione
class Config:
    DEBUG = True
    SOUND_THRESHOLD = 700
    MOTION_THRESHOLD = 5000
    DISTANCE_THRESHOLD = 50
    INHIBIT_PERIOD = 30
    FRAME_QUALITY = 35

# Variabili globali
client = None
surveillance = None
inhibit_end_period = 0

# LED per debug
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

def debug_print(msg):
    if Config.DEBUG:
        print(msg)
        if client:
            try:
                client['messages'] = f"[{get_time_string()}] {msg}"
            except:
                pass

class SurveillanceSystem:
    def __init__(self):
        self.last_frame = None
        self.base_distance = None
        self.audio_level = 0
        self.camera_enabled = False
        self.audio_enabled = False
        self.distance_enabled = False

        self.init_sensors()

    def init_sensors(self):
        debug_print("Inizializzazione sensori...")

        # Camera
        try:
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            sensor.set_vflip(False)
            sensor.set_hmirror(True)
            sensor.skip_frames(time=2000)
            self.camera_enabled = True
            debug_print("Camera inizializzata")
        except Exception as e:
            debug_print(f"Errore camera: {e}")

        # Audio
        try:
            audio.init(channels=1, frequency=16000, gain_db=24, highpass=0.9883)
            self.audio_enabled = True
            debug_print("Audio inizializzato")
        except Exception as e:
            debug_print(f"Errore audio: {e}")

        # Distanza
        try:
            self.i2c = pyb.I2C(2, pyb.I2C.MASTER, baudrate=400000)
            self.base_distance = self.read_distance()
            if self.base_distance > 0:
                self.distance_enabled = True
                debug_print(f"Sensore distanza inizializzato: {self.base_distance}mm")
        except Exception as e:
            debug_print(f"Errore sensore distanza: {e}")

    def read_distance(self):
        try:
            self.i2c.send(0x00, 0x29)
            data = self.i2c.recv(2, 0x29)
            return (data[0] << 8) | data[1]
        except:
            return 0

    def check_motion(self):
        if not self.camera_enabled:
            return False

        try:
            current_frame = sensor.snapshot()
            if self.last_frame is None:
                self.last_frame = current_frame
                return False

            diff = current_frame.difference(self.last_frame)
            stats = diff.statistics()
            motion_level = stats.mean()
            self.last_frame = current_frame

            if motion_level > client['motion_threshold']:
                debug_print(f"Movimento rilevato: {motion_level}")
                return True

        except Exception as e:
            debug_print(f"Errore motion detection: {e}")

        return False

    def check_distance(self):
        if not self.distance_enabled:
            return False

        try:
            current_distance = self.read_distance()
            diff = abs(current_distance - self.base_distance)

            if diff > client['distance_threshold']:
                debug_print(f"Distanza cambiata: {diff}mm")
                return True

        except Exception as e:
            debug_print(f"Errore distance detection: {e}")

        return False

    def capture_frame(self):
        if not self.camera_enabled:
            return None

        try:
            red_led.on()
            img = sensor.snapshot()
            img.compress(quality=Config.FRAME_QUALITY)
            data = img.compressed_bytes()
            red_led.off()
            return data
        except Exception as e:
            debug_print(f"Errore cattura frame: {e}")
            red_led.off()
            return None

def get_time_string():
    ts = time.gmtime()
    return f"{ts[0]}-{ts[1]}-{ts[2]} {ts[3]}:{ts[4]}:{ts[5]} UTC"

def audio_callback(buf):
    global surveillance, client, inhibit_end_period

    if not (surveillance and surveillance.audio_enabled and
            client and client['audio_monitoring'] and
            time.time() > inhibit_end_period):
        return

    try:
        samples = [buf[i] | (buf[i+1] << 8) for i in range(0, len(buf), 2)]
        level = sum(abs(s) for s in samples) / len(samples)

        if level > client['sound_threshold']:
            debug_print(f"Suono rilevato: {level}")
            handle_event('audio', level)

    except Exception as e:
        debug_print(f"Errore audio callback: {e}")

def handle_event(event_type, value=None):
    global client, inhibit_end_period

    try:
        green_led.on()
        frame_data = surveillance.capture_frame()

        if frame_data:
            client['current_frame'] = frame_data
            client['event_type'] = event_type
            client['last_event_time'] = time.time()

            msg = f"Evento {event_type}"
            if value:
                msg += f" (valore: {value})"
            debug_print(msg)

        inhibit_end_period = time.time() + Config.INHIBIT_PERIOD
        green_led.off()

    except Exception as e:
        debug_print(f"Errore gestione evento: {e}")
        green_led.off()

def on_global_enable_change(client, value):
    debug_print(f"Sistema {'attivato' if value else 'disattivato'}")
    if value and surveillance.audio_enabled:
        try:
            audio.start_streaming(audio_callback)
            debug_print("Streaming audio avviato")
        except Exception as e:
            debug_print(f"Errore avvio streaming audio: {e}")
    else:
        try:
            audio.stop_streaming()
            debug_print("Streaming audio fermato")
        except:
            pass

def on_camera_monitoring_change(client, value):
    debug_print(f"Monitoraggio camera {'attivato' if value else 'disattivato'}")
    if value and not surveillance.camera_enabled:
        debug_print("ATTENZIONE: Camera non disponibile")

def on_audio_monitoring_change(client, value):
    debug_print(f"Monitoraggio audio {'attivato' if value else 'disattivato'}")
    if value and not surveillance.audio_enabled:
        debug_print("ATTENZIONE: Audio non disponibile")

def on_distance_monitoring_change(client, value):
    debug_print(f"Monitoraggio distanza {'attivato' if value else 'disattivato'}")
    if value and not surveillance.distance_enabled:
        debug_print("ATTENZIONE: Sensore distanza non disponibile")

def init_cloud():
    global client

    debug_print("Inizializzazione Arduino IoT Cloud...")
    client = ArduinoCloudClient(device_id=DEVICE_ID, username=DEVICE_ID, password=SECRET_KEY)

    # Registra variabili con callback
    variables = [
        ("global_enable", False, on_global_enable_change),
        ("camera_monitoring", False, on_camera_monitoring_change),
        ("audio_monitoring", False, on_audio_monitoring_change),
        ("distance_monitoring", False, on_distance_monitoring_change),
        ("sound_threshold", 700, None),
        ("motion_threshold", 5000, None),
        ("distance_threshold", 50, None),
        ("current_frame", None, None),
        ("event_type", "", None),
        ("last_event_time", 0, None),
        ("messages", "", None),
        ("system_status", "", None)
    ]

    for var_name, default_value, callback in variables:
        try:
            debug_print(f"Registrazione {var_name}...")
            if callback:
                client.register(var_name, value=default_value, on_write=callback)
            else:
                client.register(var_name, value=default_value)
        except Exception as e:
            debug_print(f"Errore registrazione {var_name}: {e}")

    client.register(Task("wifi_connection", on_run=async_wifi_connection, interval=60.0))

    return client

def main():
    global surveillance, client

    try:
        # Inizializza sensori
        surveillance = SurveillanceSystem()

        # Connetti WiFi
        debug_print("Connessione WiFi...")
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        wifi.connect(WIFI_SSID, WIFI_PASS)

        while not wifi.isconnected():
            time.sleep(1)
        debug_print(f"WiFi connesso: {wifi.ifconfig()}")

        # Inizializza cloud
        client = init_cloud()

        # Aggiorna stato sistema
        status = []
        if surveillance.camera_enabled:
            status.append("Camera: OK")
        if surveillance.audio_enabled:
            status.append("Audio: OK")
        if surveillance.distance_enabled:
            status.append("Distanza: OK")

        status_msg = " | ".join(status) if status else "Nessun sensore attivo"
        client['system_status'] = status_msg
        client['messages'] = f"[{get_time_string()}] Sistema inizializzato - {status_msg}"

        # Avvia client
        client.start()
        debug_print("Sistema pronto!")

        # Loop principale
        while True:
            try:
                if client['global_enable']:
                    if client['camera_monitoring'] and surveillance.camera_enabled:
                        if surveillance.check_motion():
                            debug_print("Rilevato movimento")
                            handle_event('motion')

                    if client['distance_monitoring'] and surveillance.distance_enabled:
                        if surveillance.check_distance():
                            debug_print("Rilevata variazione distanza")
                            handle_event('distance')

                time.sleep_ms(100)

            except Exception as e:
                debug_print(f"Errore nel loop principale: {e}")
                time.sleep(1)

    except Exception as e:
        debug_print(f"Errore fatale: {e}")
        red_led.on()

    finally:
        if surveillance and surveillance.audio_enabled:
            try:
                audio.stop_streaming()
            except:
                pass

def check_sensor_thresholds():
    """Verifica periodica delle soglie dei sensori"""
    if 'sound_threshold' in client:
        Config.SOUND_THRESHOLD = client['sound_threshold']
    if 'motion_threshold' in client:
        Config.MOTION_THRESHOLD = client['motion_threshold']
    if 'distance_threshold' in client:
        Config.DISTANCE_THRESHOLD = client['distance_threshold']

if __name__ == "__main__":
    main()
