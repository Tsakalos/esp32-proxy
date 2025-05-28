from flask import Flask, request
import requests

app = Flask(__name__)

# Your actual Google Apps Script deployment URL
# GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbyYNT6qnNAIhtb_9cfscGDbtwG3_ohSYogX5lxrM4_KdruKmSNtSgayK61uiJU3Od-Z/exec"

GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbyYNT6qnNAIhtb_9cfscGDbtwG3_ohSYogX5lxrM4_KdruKmSNtSgayK61uiJU3Od-Z/exec"


@app.before_request
def log_request_info():
    print(f"\n📡 Received {request.method} on {request.path}", flush=True)

@app.route('/upload', methods=['POST'])
def upload():
    print("🚀 /upload route triggered", flush=True)

    raw = request.get_data(as_text=True)
    print(f"🔍 Raw received:\n{'─'*30}\n{raw}\n{'─'*30}", flush=True)

    if "image=" not in raw:
        return "❌ 'image=' not found", 400

    # isolate everything after the literal "image="
    body = raw.split("image=", 1)[1]
    # normalize CRLF → LF and split into lines
    lines = body.replace("\r\n", "\n").split("\n")

    parts = []
    i = 0
    chunk_num = 0

    while i < len(lines):
        hex_line = lines[i].strip()

        try:
            chunk_len = int(hex_line, 16)
        except ValueError:
            print(f"⚠️ Invalid hex line {i}: {hex_line}", flush=True)
            i += 1
            continue

        if chunk_len == 0:
            print("🔚 Final chunk detected", flush=True)
            break

        i += 1
        if i >= len(lines):
            print("❌ Missing data line after length", flush=True)
            break

        # Reconstruct chunk up to declared length
        chunk_data = ""
        while i < len(lines) and len(chunk_data) < chunk_len:
            chunk_data += lines[i]
            i += 1

        if len(chunk_data) != chunk_len:
            print(f"⚠️ Chunk {chunk_num} length mismatch: expected {chunk_len}, got {len(chunk_data)}", flush=True)
            continue

        print(f"⬇️ --------- Chunk {chunk_num} ---------")
        print(f"🔢 Length (hex): {hex_line}")
        print(f"📦 Payload ({len(chunk_data)} bytes): {chunk_data[:60]}{'...' if len(chunk_data) > 60 else ''}\n", flush=True)

        parts.append(chunk_data)
        chunk_num += 1

    image_data = "".join(parts).strip()
    print(f"✅ Total extracted base64 length: {len(image_data)}", flush=True)

    # Forward to Google Apps Script
    response = requests.post(
        GOOGLE_SCRIPT_URL,
        data=image_data,
        headers={"Content-Type": "text/plain"}
    )
    print(f"📤 Google responded with: {response.status_code}", flush=True)
    return "✅ Image forwarded to Google", 200

@app.route('/', methods=['GET'])
def home():
    return "✅ Flask Proxy is running."

if __name__ == '__main__':
    print("🌐 Starting Flask proxy on http://0.0.0.0:8080", flush=True)
    app.run(host='0.0.0.0', port=8080, debug=True)