from flask import Flask, request
import requests

app = Flask(__name__)

GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbyYNT6qnNAIhtb_9cfscGDbtwG3_ohSYogX5lxrM4_KdruKmSNtSgayK61uiJU3Od-Z/exec?folder=ESP32-Data-Logging"

@app.route('/upload', methods=['POST'])
def upload():
    image_data = request.form.get('image')
    if not image_data:
        return "❌ No 'image' data received.", 400

    response = requests.post(GOOGLE_SCRIPT_URL, data={'image': image_data})
    return f"✅ Forwarded to Google Apps Script. Status: {response.status_code}", response.status_code

@app.route('/')
def home():
    return "ESP32 Proxy is running!"