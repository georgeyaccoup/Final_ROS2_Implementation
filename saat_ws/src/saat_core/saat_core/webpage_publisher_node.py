#!/usr/bin/env python3
"""
webpage_publisher_node  (webpage_publisher_status)
===================================================
Serves a live SCADA-style web dashboard over HTTP using Flask.

Two pages (Section 13 / Section 14)
-------------------------------------
GET /           → Status page    (refreshes at 0.1 Hz / every 10 s)
GET /database   → Database page  (live SQLite table viewer)
GET /api/status → JSON API       (raw 13-field IoT payload)

Design reference: image_364f65.jpg (dark IIoT SCADA theme)
  - Background: #0d1117
  - Card surface: #161b22
  - Accent green: #00ff88
  - Warning amber: #f59e0b
  - Danger red:    #ef4444
  - Font: JetBrains Mono / monospace

Subscribes
----------
/iot_status      std_msgs/String   (JSON — 13 IoT fields from data_collection_node)
/speed_to_plc    saat_interfaces/SpeedCommand

Publishes
---------
(nothing — read-only ROS2 subscriber + HTTP server)
"""

import json
import os
import sqlite3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from saat_interfaces.msg import SpeedCommand

try:
    from flask import Flask, jsonify, render_template_string
    _FLASK_OK = True
except ImportError:
    _FLASK_OK = False

# ── HTML templates ────────────────────────────────────────────────────────

_STATUS_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta http-equiv="refresh" content="10">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SAAT — Pear Sorting SCADA</title>
<style>
  :root {
    --bg:        #0d1117;
    --surface:   #161b22;
    --border:    #30363d;
    --green:     #00ff88;
    --amber:     #f59e0b;
    --red:       #ef4444;
    --blue:      #3b82f6;
    --text:      #e6edf3;
    --muted:     #8b949e;
    --font:      'JetBrains Mono', 'Courier New', monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    min-height: 100vh;
    padding: 24px;
  }

  /* ── Top bar ─────────────────────────────────────────────────── */
  .topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid var(--border);
    padding-bottom: 16px;
    margin-bottom: 24px;
  }
  .topbar .brand { font-size: 1.2rem; font-weight: 700; color: var(--green); letter-spacing: 2px; }
  .topbar .tagline { font-size: 0.7rem; color: var(--muted); margin-top: 2px; }
  .status-pill {
    padding: 4px 14px;
    border-radius: 999px;
    font-size: 0.72rem;
    font-weight: 600;
    letter-spacing: 1px;
    background: rgba(0,255,136,0.12);
    color: var(--green);
    border: 1px solid var(--green);
    animation: pulse 2s infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.6} }

  /* ── KPI cards ───────────────────────────────────────────────── */
  .kpi-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
    gap: 16px;
    margin-bottom: 24px;
  }
  .kpi-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 18px 20px;
    position: relative;
    overflow: hidden;
  }
  .kpi-card::before {
    content: '';
    position: absolute;
    top: 0; left: 0;
    width: 3px; height: 100%;
    background: var(--accent, var(--green));
  }
  .kpi-label { font-size: 0.65rem; color: var(--muted); text-transform: uppercase; letter-spacing: 1.5px; }
  .kpi-value { font-size: 2rem; font-weight: 700; color: var(--accent, var(--green)); margin-top: 6px; }
  .kpi-sub   { font-size: 0.65rem; color: var(--muted); margin-top: 4px; }

  /* ── Zone grid ───────────────────────────────────────────────── */
  .section-title {
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: var(--muted);
    margin-bottom: 12px;
    padding-bottom: 6px;
    border-bottom: 1px solid var(--border);
  }
  .zone-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 12px;
    margin-bottom: 24px;
  }
  .zone-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 14px;
  }
  .zone-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 10px;
  }
  .zone-id { font-size: 1.1rem; font-weight: 700; color: var(--text); }
  .zone-badge {
    font-size: 0.62rem;
    padding: 2px 8px;
    border-radius: 4px;
    font-weight: 600;
  }
  .badge-ok   { background: rgba(0,255,136,0.15); color: var(--green); }
  .badge-bad  { background: rgba(239,68,68,0.15);  color: var(--red);   }
  .badge-idle { background: rgba(139,148,158,0.15); color: var(--muted); }
  .zone-row {
    display: flex;
    justify-content: space-between;
    font-size: 0.7rem;
    color: var(--muted);
    margin-top: 5px;
  }
  .zone-row span:last-child { color: var(--text); }

  /* ── Belt speed card ─────────────────────────────────────────── */
  .belt-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
    margin-bottom: 24px;
  }
  .belt-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 20px;
  }
  .belt-title { font-size: 0.65rem; color: var(--muted); text-transform: uppercase; letter-spacing: 1.5px; }
  .belt-value { font-size: 1.6rem; font-weight: 700; color: var(--blue); margin-top: 6px; }
  .volt-bar-wrap { margin-top: 10px; background: var(--bg); border-radius: 4px; height: 8px; overflow: hidden; }
  .volt-bar { height: 8px; border-radius: 4px; background: linear-gradient(90deg, var(--green), var(--blue)); transition: width 0.4s ease; }

  /* ── Footer ──────────────────────────────────────────────────── */
  .footer {
    text-align: center;
    font-size: 0.62rem;
    color: var(--muted);
    padding-top: 16px;
    border-top: 1px solid var(--border);
    margin-top: 8px;
  }
  a { color: var(--green); text-decoration: none; }
</style>
</head>
<body>

<!-- ── Top bar ─────────────────────────────────────────────── -->
<div class="topbar">
  <div>
    <div class="brand">🍐 SAAT — PEAR SORTING LINE</div>
    <div class="tagline">IIoT SCADA SYSTEM &nbsp;·&nbsp; Solar-Powered Automated Agricultural Technology</div>
  </div>
  <div class="status-pill">● SYSTEM RUNNING</div>
</div>

<!-- ── KPI row ─────────────────────────────────────────────── -->
<div class="kpi-grid">
  <div class="kpi-card" style="--accent:var(--green)">
    <div class="kpi-label">Accepted</div>
    <div class="kpi-value">{{ d.total_accepted }}</div>
    <div class="kpi-sub">Batch: {{ d.batch_accepted }}</div>
  </div>
  <div class="kpi-card" style="--accent:var(--red)">
    <div class="kpi-label">Rejected</div>
    <div class="kpi-value">{{ d.total_rejected }}</div>
    <div class="kpi-sub">Batch: {{ d.batch_rejected }}</div>
  </div>
  <div class="kpi-card" style="--accent:var(--amber)">
    <div class="kpi-label">Avg Mass</div>
    <div class="kpi-value">{{ "%.1f"|format(d.avg_mass_g) }}<span style="font-size:1rem">g</span></div>
    <div class="kpi-sub">Vol: {{ "%.1f"|format(d.avg_volume_cm3) }} cm³</div>
  </div>
  <div class="kpi-card" style="--accent:var(--blue)">
    <div class="kpi-label">Belt Speed</div>
    <div class="kpi-value">{{ "%.2f"|format(d.belt_speed_ms) }}<span style="font-size:1rem">m/s</span></div>
    <div class="kpi-sub">Reference setpoint</div>
  </div>
  <div class="kpi-card" style="--accent:#a78bfa">
    <div class="kpi-label">Big Pears</div>
    <div class="kpi-value">{{ d.total_big }}</div>
    <div class="kpi-sub">Small: {{ d.total_small }}</div>
  </div>
  <div class="kpi-card" style="--accent:var(--muted)">
    <div class="kpi-label">Last Pear ID</div>
    <div class="kpi-value" style="font-size:1.1rem">{{ d.latest_pear_id }}</div>
    <div class="kpi-sub">Zone: {{ d.latest_zone_id }}</div>
  </div>
</div>

<!-- ── Belt speed cards ─────────────────────────────────────── -->
<div class="section-title">Conveyor Voltages (PWM → PLC)</div>
<div class="belt-grid">
  <div class="belt-card">
    <div class="belt-title">Conv 1 — Loading Belt (175 cm)</div>
    <div class="belt-value">{{ "%.2f"|format(d.conv1_voltage) }} V</div>
    <div class="volt-bar-wrap">
      <div class="volt-bar" style="width:{{ [(d.conv1_voltage/3.3*100), 100]|min }}%"></div>
    </div>
  </div>
  <div class="belt-card">
    <div class="belt-title">Conv 2 — Packing Belt (115 cm)</div>
    <div class="belt-value">{{ "%.2f"|format(d.conv2_voltage) }} V</div>
    <div class="volt-bar-wrap">
      <div class="volt-bar" style="width:{{ [(d.conv2_voltage/3.3*100), 100]|min }}%"></div>
    </div>
  </div>
</div>

<!-- ── Zone status grid ──────────────────────────────────────── -->
<div class="section-title">Zone Status — A1 · A2 · A3 · B1 · B2 · B3</div>
<div class="zone-grid">
{% for zone in ['A1','A2','A3','B1','B2','B3'] %}
  {% set z = zones.get(zone, {}) %}
  <div class="zone-card">
    <div class="zone-header">
      <span class="zone-id">{{ zone }}</span>
      {% if z.get('is_infected') %}
        <span class="zone-badge badge-bad">REJECTED</span>
      {% elif z.get('pear_detected') %}
        <span class="zone-badge badge-ok">ACCEPTED</span>
      {% else %}
        <span class="zone-badge badge-idle">IDLE</span>
      {% endif %}
    </div>
    <div class="zone-row"><span>Infection</span><span>{{ "%.1f"|format(z.get('infection_ratio',0)*100) }}%</span></div>
    <div class="zone-row"><span>Area</span><span>{{ z.get('pear_area_px', 0)|int }} px²</span></div>
  </div>
{% endfor %}
</div>

<div class="footer">
  Updated: {{ ts }} &nbsp;·&nbsp; Refresh: 10 s &nbsp;·&nbsp;
  <a href="/database">Database View →</a> &nbsp;·&nbsp;
  <a href="/api/status">JSON API →</a>
</div>

</body>
</html>"""

_DB_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>SAAT — Database</title>
<style>
  :root { --bg:#0d1117; --surface:#161b22; --border:#30363d;
          --green:#00ff88; --red:#ef4444; --text:#e6edf3; --muted:#8b949e; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text);
         font-family:'JetBrains Mono','Courier New',monospace; padding:24px; }
  h2 { color:var(--green); margin-bottom:16px; font-size:1rem; letter-spacing:2px; }
  .back { color:var(--green); text-decoration:none; font-size:0.75rem; display:block; margin-bottom:16px; }
  table { width:100%; border-collapse:collapse; font-size:0.68rem; }
  th { background:var(--surface); color:var(--muted); text-transform:uppercase;
       letter-spacing:1px; padding:8px 12px; text-align:left; border-bottom:2px solid var(--border); }
  td { padding:7px 12px; border-bottom:1px solid var(--border); }
  tr:hover td { background:var(--surface); }
  .ok  { color:var(--green); font-weight:700; }
  .bad { color:var(--red);   font-weight:700; }
</style>
</head>
<body>
<a class="back" href="/">← Back to Status</a>
<h2>🗄 PEAR RECORDS DATABASE</h2>
<table>
<thead><tr>
  <th>Pear ID</th><th>Zone</th><th>Status</th><th>Cat.</th>
  <th>Inf %</th><th>Area px²</th><th>Vol cm³</th><th>Mass g</th>
  <th>Speed m/s</th><th>Timestamp</th>
</tr></thead>
<tbody>
{% for r in records %}
<tr>
  <td>{{ r.pear_id }}</td>
  <td>{{ r.zone_id }}</td>
  <td class="{{ 'ok' if r.pear_status=='ACCEPTED' else 'bad' }}">{{ r.pear_status }}</td>
  <td>{{ r.pear_category }}</td>
  <td>{{ "%.1f"|format(r.infection_ratio*100) }}%</td>
  <td>{{ r.pear_surface_area_px|int }}</td>
  <td>{{ "%.2f"|format(r.pear_volume_cm3) }}</td>
  <td>{{ "%.1f"|format(r.pear_mass_g) }}</td>
  <td>{{ "%.3f"|format(r.belt_speed_ms) }}</td>
  <td>{{ r.timestamp|int }}</td>
</tr>
{% endfor %}
</tbody>
</table>
</body>
</html>"""


# ── ROS2 Node ──────────────────────────────────────────────────────────────
class WebpagePublisherNode(Node):
    """Hosts the live SCADA dashboard and database viewer."""

    def __init__(self):
        super().__init__('webpage_publisher_node')

        self.declare_parameter('host',           '0.0.0.0')
        self.declare_parameter('port',           8080)
        self.declare_parameter('update_rate_hz', 0.1)
        self.declare_parameter('db_path', '/saat_data/saat_records.db')

        self._host    = self.get_parameter('host').value
        self._port    = self.get_parameter('port').value
        self._db_path = self.get_parameter('db_path').value

        # ── Shared state (updated by ROS2 callbacks) ──────────────────────
        self._lock      = threading.Lock()
        self._iot_data  = {
            'latest_pear_id': 'N/A', 'latest_zone_id': 'N/A',
            'total_accepted': 0,     'total_rejected': 0,
            'total_big': 0,          'total_small': 0,
            'batch_accepted': 0,     'batch_rejected': 0,
            'avg_volume_cm3': 0.0,   'avg_mass_g': 0.0,
            'belt_speed_ms': 0.0,    'timestamp': 0.0,
            'conv1_voltage': 0.1,    'conv2_voltage': 3.2,
        }
        # Per-zone latest detection state for the zone grid
        self._zone_data: dict[str, dict] = {z: {} for z in
                                             ['A1','A2','A3','B1','B2','B3']}

        # ── Subscribers ───────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=1)
        self.create_subscription(String, '/iot_status',    self._iot_cb,   qos)
        self.create_subscription(SpeedCommand, '/speed_to_plc', self._speed_cb, qos)

        # Subscribe to each zone's detection for the zone grid
        for zone in ['A1','A2','A3','B1','B2','B3']:
            self.create_subscription(
                String,
                f'/{zone}/detection_summary',   # lightweight JSON from vision_node
                self._make_zone_cb(zone),
                qos
            )

        # ── Flask app ─────────────────────────────────────────────────────
        if not _FLASK_OK:
            self.get_logger().error('Flask not installed. Run: pip3 install flask')
            return

        self._app = Flask(__name__)
        self._register_routes()

        flask_thread = threading.Thread(
            target=self._run_flask, daemon=True, name='flask_server'
        )
        flask_thread.start()
        self.get_logger().info(
            f'SCADA dashboard at http://{self._host}:{self._port}'
        )

    # ── ROS2 callbacks ────────────────────────────────────────────────────
    def _iot_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._iot_data.update(data)
        except json.JSONDecodeError:
            pass

    def _speed_cb(self, msg: SpeedCommand) -> None:
        with self._lock:
            self._iot_data['conv1_voltage'] = msg.conv1_voltage
            self._iot_data['conv2_voltage'] = msg.conv2_voltage
            self._iot_data['belt_speed_ms'] = msg.reference_speed_ms

    def _make_zone_cb(self, zone: str):
        def _cb(msg: String):
            try:
                data = json.loads(msg.data)
                with self._lock:
                    self._zone_data[zone] = data
            except json.JSONDecodeError:
                pass
        return _cb

    # ── Flask routes ──────────────────────────────────────────────────────
    def _register_routes(self) -> None:
        app = self._app

        @app.route('/')
        def status():
            with self._lock:
                d     = dict(self._iot_data)
                zones = dict(self._zone_data)
            ts = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d.get('timestamp', 0)))
            return render_template_string(_STATUS_HTML, d=d, zones=zones, ts=ts)

        @app.route('/database')
        def database():
            records = self._fetch_records()
            return render_template_string(_DB_HTML, records=records)

        @app.route('/api/status')
        def api_status():
            with self._lock:
                return jsonify(self._iot_data)

    def _fetch_records(self) -> list[dict]:
        """Fetch the 200 most-recent pear records from SQLite for the DB page."""
        if not os.path.exists(self._db_path):
            return []
        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            cur  = conn.execute(
                'SELECT * FROM pear_records ORDER BY timestamp DESC LIMIT 200'
            )
            rows = [dict(r) for r in cur.fetchall()]
            conn.close()
            return rows
        except sqlite3.Error:
            return []

    def _run_flask(self) -> None:
        self._app.run(
            host=self._host,
            port=self._port,
            debug=False,
            use_reloader=False,
            threaded=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = WebpagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
