import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
import serial
import time
import warnings
import smtplib
import threading
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

warnings.filterwarnings("ignore")

# ---------------- EMAIL CONFIG ----------------
SENDER_EMAIL = "syedafridahamed@gmail.com"
SENDER_PASSWORD = "vdxv uhee zqyy uqem"   # Gmail app password
RECEIVER_EMAIL = "syedafrid406@gmail.com"

last_email_time = 0
EMAIL_INTERVAL = 300   # seconds (5 min) to avoid spamming

def send_email(subject, body):
    try:
        msg = MIMEMultipart()
        msg["From"] = SENDER_EMAIL
        msg["To"] = RECEIVER_EMAIL
        msg["Subject"] = subject
        msg.attach(MIMEText(body, "plain"))

        server = smtplib.SMTP("smtp.gmail.com", 587)
        server.starttls()
        server.login(SENDER_EMAIL, SENDER_PASSWORD)
        server.send_message(msg)
        server.quit()
        print("📧 Email sent successfully!")
    except Exception as e:
        print("❌ Error sending email:", e)

# ---------------- SERIAL ----------------
ser = serial.Serial('COM3', baudrate=115200)  # Adjust COM port
print("Serial connection opened successfully!")

# ---------------- LOAD DATASET ----------------
data = pd.read_excel("Health_Fall_Samples.xlsx", engine="openpyxl")

# Feature columns
feature_bpm = data['bpm']
feature_spo2 = data['spo2']
feature_temp = data['temp']
feature_angleX = data['angleX']
feature_soundState = data['soundState']

# Label columns
label_bpm        = data['Label_bpm']
label_spo2       = data['Label_spo2']
label_temp       = data['Label_temp']
label_angleX     = data['Label_angleX']
label_soundState = data['Label_soundState']

# Train Random Forest models
def train_model(X, y):
    X_train, _, y_train, _ = train_test_split(X, y, test_size=0.2, random_state=42)
    model = RandomForestClassifier(random_state=42)
    model.fit(X_train.values.reshape(-1, 1), y_train)
    return model

rf_model_bpm = train_model(feature_bpm, label_bpm)
rf_model_spo2 = train_model(feature_spo2, label_spo2)
rf_model_temp = train_model(feature_temp, label_temp)
rf_model_angleX = train_model(feature_angleX, label_angleX)
rf_model_soundState = train_model(feature_soundState, label_soundState)

# ---------------- READ SENSOR DATA ----------------
def readData():
    serial_data = ser.readline().decode(errors="ignore").strip()
    while not serial_data.startswith('a'):
        serial_data = ser.readline().decode(errors="ignore").strip()

    time.sleep(0.2)  # allow packet completion

    print("\n----------------------------")
    print("     -= Data Received =- ")
    print("----------------------------\n")
    print("Data:", serial_data, "\n")

    try:
        val_bpm = float(serial_data.split("a")[1].split("b")[0])
        val_spo2 = float(serial_data.split("b")[1].split("c")[0])
        val_temp = float(serial_data.split("c")[1].split("d")[0])
        val_angleX = float(serial_data.split("d")[1].split("e")[0])
        val_soundState = int(serial_data.split("e")[1].split("f")[0])
    except Exception as e:
        print("❌ Parsing error:", e)
        return None

    print(f"bpm        : {val_bpm}")
    print(f"spo2       : {val_spo2}")
    print(f"temp       : {val_temp}")
    print(f"angleX     : {val_angleX}")
    print(f"soundState : {val_soundState}")

    return val_bpm, val_spo2, val_temp, val_angleX, val_soundState

# ---------------- MAIN LOOP ----------------
while True:
    input_data = readData()
    if input_data is None:
        continue

    feature_bpm_val, feature_spo2_val, feature_temp_val, feature_angleX_val, feature_soundState_val = input_data

    rf_prediction_bpm = rf_model_bpm.predict([[feature_bpm_val]])[0]
    rf_prediction_spo2 = rf_model_spo2.predict([[feature_spo2_val]])[0]
    rf_prediction_temp = rf_model_temp.predict([[feature_temp_val]])[0]
    rf_prediction_angleX = rf_model_angleX.predict([[feature_angleX_val]])[0]
    rf_prediction_soundState = rf_model_soundState.predict([[feature_soundState_val]])[0]

    print("\n----------------------------")
    print("RF-predictions")
    print("----------------------------\n")
    print(f'bpm        : {rf_prediction_bpm}')
    print(f'spo2       : {rf_prediction_spo2}')
    print(f'temp       : {rf_prediction_temp}')
    print(f'angleX     : {rf_prediction_angleX}')
    print(f'soundState : {rf_prediction_soundState}')

    # Health condition
    if rf_prediction_spo2 == 1 or rf_prediction_temp == 1:
        Health_Fall_Samples = "not healthy"
    else:
        max_class = max(rf_prediction_bpm, rf_prediction_angleX, rf_prediction_soundState)
        quality_labels = ["healthy", "alert", "warning", "critical"]
        Health_Fall_Samples = quality_labels[max_class]

    print("\n----------------------------")
    print(f"Health_Fall: {Health_Fall_Samples}")
    print("----------------------------\n")

    # Send values via serial
    values_string = (
        f"t{rf_prediction_bpm}u{rf_prediction_spo2}v{rf_prediction_temp}"
        f"w{rf_prediction_angleX}x{rf_prediction_soundState}y{Health_Fall_Samples}"
    )
    ser.write(bytes(values_string, 'utf-8'))

    time.sleep(0.2)  # small pause for serial

    print("Data transmission completed\n")

    # ---------------- EMAIL ALERT (non-blocking + throttled) ----------------
    if Health_Fall_Samples == "not healthy":
        if time.time() - last_email_time > EMAIL_INTERVAL:  # throttle
            subject = "⚠️ Sleep Monitoring Alert: Not Healthy"
            body = (
                f"Patient health condition is NOT HEALTHY.\n\n"
                f"Readings:\n"
                f" - BPM: {feature_bpm_val}\n"
                f" - SpO2: {feature_spo2_val}\n"
                f" - Temp: {feature_temp_val}\n"
                f" - AngleX: {feature_angleX_val}\n"
                f" - SoundState: {feature_soundState_val}\n\n"
                f"Prediction: {Health_Fall_Samples}"
            )
            threading.Thread(target=send_email, args=(subject, body), daemon=True).start()
            last_email_time = time.time()
        else:
            print("⏳ Skipping email (recently sent)")
    else:
        print("✅ Health status is okay, no email sent.")
