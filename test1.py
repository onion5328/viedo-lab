import cv2
import time
import threading
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# 引入硬體驅動
from LOBOROBOT2 import LOBOROBOT
import libcamera
from picamera2 import Picamera2

# ==========================================
# 參數設定區
# ==========================================
WW, HH = 320, 240
SCAN_Y = HH - 60
IDEAL_DIST = 110
CAM_ANGLE_X = 90
CAM_ANGLE_Y = 20
DEFAULT_SPEED = 15
GAIN = 0.35
CANNY_MIN = 70
CANNY_MAX = 140

# ==========================================
# 初始化全域變數
# ==========================================
global_speed = DEFAULT_SPEED
is_running = False        
processed_frame = None    
frame_lock = threading.Lock()
l_ofs, r_ofs = 0, 0       

clbrobot = LOBOROBOT()
picamera = Picamera2()

# ==========================================
# 核心功能：影像辨識與邏輯
# ==========================================
def get_canny(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blurred, CANNY_MIN, CANNY_MAX)
    kernel = np.ones((5, 5), np.uint8)
    canny = cv2.dilate(canny, kernel, iterations=1)
    return canny

def process_lane_logic(img):
    global is_running, l_ofs, r_ofs
    canny = get_canny(img)
    debug_view = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
    center_x = WW // 2
    left_x, right_x = -1, -1
    for x in range(center_x, 0, -1):
        if canny[SCAN_Y, x] > 127:
            left_x = x
            break
    for x in range(center_x, WW):
        if canny[SCAN_Y, x] > 127:
            right_x = x
            break

    steering = 0
    detected_type = "None"
    if left_x != -1 and right_x != -1:
        lane_center = (left_x + right_x) // 2
        steering = lane_center - center_x
        detected_type = "Dual"
        cv2.circle(debug_view, (left_x, SCAN_Y), 5, (0, 255, 0), -1)
        cv2.circle(debug_view, (right_x, SCAN_Y), 5, (0, 255, 0), -1)
        cv2.circle(debug_view, (lane_center, SCAN_Y), 4, (0, 0, 255), -1)
    elif left_x != -1:
        current_dist = center_x - left_x
        steering = IDEAL_DIST - current_dist
        detected_type = "Left Only"
        cv2.circle(debug_view, (left_x, SCAN_Y), 5, (255, 255, 0), -1)
    elif right_x != -1:
        current_dist = right_x - center_x
        steering = -(IDEAL_DIST - current_dist)
        detected_type = "Right Only"
        cv2.circle(debug_view, (right_x, SCAN_Y), 5, (255, 0, 255), -1)
    else:
        steering = 0
        detected_type = "LOST"
        cv2.putText(debug_view, "LOST TRACK", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    delta = int(steering * GAIN)
    limit = global_speed + 20
    delta = max(min(delta, limit), -limit)
    l_ofs = delta
    r_ofs = -delta

    cv2.line(debug_view, (0, SCAN_Y), (WW, SCAN_Y), (100, 100, 100), 1)
    cv2.line(debug_view, (center_x, SCAN_Y - 10), (center_x, SCAN_Y + 10),
             (255, 255, 255), 1)
    cv2.putText(debug_view, f"Mode:{detected_type} Str:{int(steering)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    return debug_view

# ==========================================
# 主循環
# ==========================================
def main_loop():
    global processed_frame, is_running
    config = picamera.create_preview_configuration(
        main={"format": "RGB888", "size": (WW, HH)},
        raw={"format": "SRGGB12", "size": (WW, HH)},
    )
    config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    picamera.configure(config)
    picamera.start()

    print("Camera Started (Mask Mode).")
    while True:
        frame = picamera.capture_array()
        try:
            mask_view = process_lane_logic(frame.copy())
        except Exception as e:
            print(f"Error: {e}")
            mask_view = frame
        with frame_lock:
            processed_frame = mask_view
        if is_running:
            clbrobot.move_with_offset(global_speed, l_ofs, r_ofs, 0.05)
        time.sleep(0.03)

t = threading.Thread(target=main_loop, daemon=True)
t.start()

# ==========================================
# Flask 伺服器（淺色網頁版）
# ==========================================
app = Flask(__name__)

INDEX_HTML = """
<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pi 自走車控制面板</title>
<style>
body {
  font-family: "Noto Sans TC", "Segoe UI", sans-serif;
  background: #f8fafc;
  color: #222;
  margin: 0;
  padding: 0;
}
header {
  background: #e3f2fd;
  color: #0d47a1;
  text-align: center;
  padding: 15px 0;
  font-size: 22px;
  font-weight: bold;
  letter-spacing: 0.5px;
  box-shadow: 0 2px 6px rgba(0,0,0,0.1);
}
main {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  align-items: flex-start;
  gap: 30px;
  padding: 25px;
}
.video-box {
  flex: 2;
  min-width: 320px;
  text-align: center;
}
.video-box img {
  width: 95%;
  border-radius: 10px;
  border: 2px solid #ccc;
  box-shadow: 0 3px 10px rgba(0,0,0,0.1);
}
.status {
  margin-top: 10px;
  font-size: 1.1em;
  color: #1976d2;
  font-weight: 600;
}
.control-panel {
  flex: 1;
  min-width: 250px;
  background: #fff;
  border-radius: 10px;
  padding: 20px;
  box-shadow: 0 3px 10px rgba(0,0,0,0.1);
}
.control-panel h3 {
  margin-top: 0;
  color: #1565c0;
  text-align: center;
}
button {
  width: 100%;
  padding: 12px;
  margin: 8px 0;
  border: none;
  border-radius: 8px;
  font-size: 18px;
  font-weight: bold;
  color: white;
  cursor: pointer;
  transition: 0.2s;
}
.start { background: #4caf50; }
.stop { background: #f44336; }
.adj { background: #2196f3; }
button:hover { filter: brightness(1.1); }

.speedbar {
  margin-top: 15px;
  text-align: center;
}
.speedbar input {
  width: 100%;
}
footer {
  text-align: center;
  color: #666;
  font-size: 0.9em;
  margin: 15px 0 25px;
}
</style>
</head>
<body>
<header>🚗 Pi 自走車控制面板</header>

<main>
  <div class="video-box">
    <img src="/video_feed" alt="Live Stream">
    <div class="status" id="status_text">狀態：待機中</div>
  </div>

  <div class="control-panel">
    <h3>控制區</h3>
    <button class="start" onclick="cmd('start')">▶ 啟動</button>
    <button class="stop" onclick="cmd('stop')">■ 停止</button>
    <button class="adj" onclick="cmd('speed_up')">＋ 提高速度</button>
    <button class="adj" onclick="cmd('speed_down')">－ 降低速度</button>
    <div class="speedbar">
      <label>目前速度：<span id="spd_val">15</span></label>
      <input type="range" min="10" max="80" value="15" id="spd_slider" disabled>
    </div>
  </div>
</main>

<footer>© 2025 Raspberry Pi AutoCar — 淺色版控制介面</footer>

<script>
function cmd(c){
  fetch('/api/' + c, {method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.speed){ 
      document.getElementById('spd_val').innerText = d.speed;
      document.getElementById('spd_slider').value = d.speed;
    }
    if(d.status)
      document.getElementById('status_text').innerText = "狀態：" + d.status;
  });
}
</script>
</body>
</html>
"""

def generate_mjpeg():
    while True:
        with frame_lock:
            if processed_frame is None:
                time.sleep(0.01)
                continue
            ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR),
                                     [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
        time.sleep(0.05)

@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/start", methods=["POST"])
def api_start():
    global is_running
    is_running = True
    return jsonify({"status": "RUNNING", "speed": global_speed})

@app.route("/api/stop", methods=["POST"])
def api_stop():
    global is_running
    is_running = False
    clbrobot.t_stop(0.1)
    return jsonify({"status": "STOPPED", "speed": global_speed})

@app.route("/api/speed_up", methods=["POST"])
def api_speed_up():
    global global_speed
    global_speed = min(global_speed + 5, 80)
    return jsonify({"speed": global_speed})

@app.route("/api/speed_down", methods=["POST"])
def api_speed_down():
    global global_speed
    global_speed = max(global_speed - 5, 10)
    return jsonify({"speed": global_speed})

# ==========================================
# 主程式入口
# ==========================================
def cleanup():
    print("Exiting...")
    clbrobot.t_stop(0.1)
    try:
        picamera.stop()
    except:
        pass

if __name__ == "__main__":
    clbrobot.set_servo_angle(10, CAM_ANGLE_X, 0.5)
    clbrobot.set_servo_angle(9, CAM_ANGLE_Y, 0.5)
    time.sleep(1)
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        cleanup()
