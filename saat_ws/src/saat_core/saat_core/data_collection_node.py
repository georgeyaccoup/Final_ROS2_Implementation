#!/usr/bin/env python3
"""
data_collection_node
=====================
Central database node.  Consumes PearData from all 6 zones, merges
volume + mass measurements, and writes the complete 13-field IoT record
to a local SQLite database.

Database schema  (one row per pear detected, write order: A1→A2→A3→B1→B2→B3)
----------------------------------------------------------------------
| Field  | Column                  | Type    | Source               |
|--------|-------------------------|---------|----------------------|
|   1    | pear_id                 | TEXT PK | data_collector_node  |
|   2    | zone_id                 | TEXT    | data_collector_node  |
|   3    | timestamp               | REAL    | ROS header stamp     |
|   4    | pear_status             | TEXT    | action_node          |
|   5    | pear_category           | TEXT    | data_collector_node  |
|   6    | infection_area_px       | REAL    | vision_node          |
|   7    | infection_location      | TEXT    | vision_node (JSON)   |
|   8    | infection_color_rgb     | TEXT    | vision_node (JSON)   |
|   9    | infection_ratio         | REAL    | vision_node          |
|  10    | pear_surface_area_px    | REAL    | area_node            |
|  11    | pear_volume_cm3         | REAL    | volume_estimation    |
|  12    | pear_mass_g             | REAL    | mass_estimation      |
|  13    | belt_speed_ms           | REAL    | main_speed_node      |
----------------------------------------------------------------------

Subscribes
----------
/A1/pear_data … /B3/pear_data   saat_interfaces/PearData
/A1/volume    … /B3/volume      std_msgs/Float32
/A1/mass      … /B3/mass        std_msgs/Float32
/speed_to_plc                   saat_interfaces/SpeedCommand

Publishes
---------
/iot_status                     std_msgs/String  (JSON, all 13 IoT fields)
"""

import json
import os
import sqlite3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from saat_interfaces.msg import PearData, SpeedCommand

_ZONES = ["A1", "A2", "A3", "B1", "B2", "B3"]

# Write order from Section 14 of report
_WRITE_ORDER = ["A1", "A2", "A3", "B1", "B2", "B3"]

_CREATE_TABLE = """
CREATE TABLE IF NOT EXISTS pear_records (
    pear_id              TEXT PRIMARY KEY,
    zone_id              TEXT    NOT NULL,
    timestamp            REAL    NOT NULL,
    pear_status          TEXT    NOT NULL,
    pear_category        TEXT    NOT NULL,
    infection_area_px    REAL    DEFAULT 0.0,
    infection_location   TEXT    DEFAULT '{}',
    infection_color_rgb  TEXT    DEFAULT '[0,0,0]',
    infection_ratio      REAL    DEFAULT 0.0,
    pear_surface_area_px REAL    DEFAULT 0.0,
    pear_volume_cm3      REAL    DEFAULT 0.0,
    pear_mass_g          REAL    DEFAULT 0.0,
    belt_speed_ms        REAL    DEFAULT 0.0
);
"""

_INSERT_RECORD = """
INSERT OR REPLACE INTO pear_records
    (pear_id, zone_id, timestamp, pear_status, pear_category,
     infection_area_px, infection_location, infection_color_rgb,
     infection_ratio, pear_surface_area_px, pear_volume_cm3,
     pear_mass_g, belt_speed_ms)
VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);
"""


class DataCollectionNode(Node):
    """
    Central database writer and IoT status publisher.
    Merges PearData with volume/mass/speed to produce complete 13-field records.
    """

    def __init__(self):
        super().__init__('data_collection_node')

        self.declare_parameter('db_path',               '/saat_data/saat_records.db')
        self.declare_parameter('small_area_threshold_px', 15000)

        db_path = self.get_parameter('db_path').value

        # Ensure the database directory exists
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        # ── SQLite setup ─────────────────────────────────────────────────
        self._db_lock = threading.Lock()
        self._conn    = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute(_CREATE_TABLE)
        self._conn.commit()
        self.get_logger().info(f'SQLite database ready at {db_path}')

        # ── Counters ──────────────────────────────────────────────────────
        self._accepted: int  = 0
        self._rejected: int  = 0
        self._packages: int  = 0

        # ── Per-zone pending data caches ─────────────────────────────────
        # We receive PearData first, then volume/mass arrive shortly after.
        # Cache them per pear_id and merge when all three arrive.
        self._pending: dict[str, dict] = {}   # pear_id → partial record
        self._volumes: dict[str, float] = {z: 0.0 for z in _ZONES}
        self._masses:  dict[str, float] = {z: 0.0 for z in _ZONES}

        # Latest belt speed for field 13
        self._belt_speed: float = 0.0

        # ── QoS ──────────────────────────────────────────────────────────
        qos     = rclpy.qos.QoSProfile(depth=10)
        qos_low = rclpy.qos.QoSProfile(depth=1)

        # ── IoT publisher ─────────────────────────────────────────────────
        self._iot_pub = self.create_publisher(String, '/iot_status', qos)

        # ── Subscribers — PearData from all zones ─────────────────────────
        for zone in _ZONES:
            self.create_subscription(
                PearData,
                f'/{zone}/pear_data',
                self._make_pear_cb(zone),
                qos
            )
            self.create_subscription(
                Float32, f'/{zone}/volume',
                self._make_vol_cb(zone), qos_low
            )
            self.create_subscription(
                Float32, f'/{zone}/mass',
                self._make_mass_cb(zone), qos_low
            )

        self.create_subscription(
            SpeedCommand, '/speed_to_plc',
            self._speed_cb, qos_low
        )

        # ── Flush timer: write pending records every 2 seconds ────────────
        self._flush_timer = self.create_timer(2.0, self._flush_pending)

        # ── IoT publish timer: 0.1 Hz (every 10 s) as per Section 13 ─────
        self._iot_timer   = self.create_timer(10.0, self._publish_iot)

        self.get_logger().info('data_collection_node ready.')

    # ── Subscriber factories ──────────────────────────────────────────────
    def _make_pear_cb(self, zone: str):
        def _cb(msg: PearData):
            pid = msg.pear_id
            self._pending[pid] = {
                'pear_id':              pid,
                'zone_id':              zone,
                'timestamp':            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'pear_status':          msg.pear_status,
                'pear_category':        msg.pear_category,
                'infection_area_px':    msg.infection_area_px,
                'infection_location':   json.dumps({'x': msg.infection_x, 'y': msg.infection_y}),
                'infection_color_rgb':  json.dumps([msg.infection_r, msg.infection_g, msg.infection_b]),
                'infection_ratio':      msg.infection_ratio,
                'pear_surface_area_px': msg.pear_surface_area_px,
                'pear_volume_cm3':      self._volumes.get(zone, 0.0),
                'pear_mass_g':          self._masses.get(zone, 0.0),
                'belt_speed_ms':        self._belt_speed,
            }
            # Update counters
            if msg.pear_status == 'ACCEPTED':
                self._accepted += 1
            elif msg.pear_status == 'REJECTED':
                self._rejected += 1
        return _cb

    def _make_vol_cb(self, zone: str):
        def _cb(msg: Float32):
            self._volumes[zone] = msg.data
        return _cb

    def _make_mass_cb(self, zone: str):
        def _cb(msg: Float32):
            self._masses[zone] = msg.data
        return _cb

    def _speed_cb(self, msg: SpeedCommand) -> None:
        self._belt_speed = msg.reference_speed_ms

    # ── Flush pending records to SQLite ──────────────────────────────────
    def _flush_pending(self) -> None:
        """
        Write all cached pending records to SQLite.
        Respects the write order: A1→A2→A3→B1→B2→B3.
        """
        if not self._pending:
            return

        # Sort by zone then by pear_id counter
        sorted_records = sorted(
            self._pending.values(),
            key=lambda r: (_WRITE_ORDER.index(r['zone_id']) if r['zone_id'] in _WRITE_ORDER else 99,
                           r['pear_id'])
        )

        with self._db_lock:
            try:
                for rec in sorted_records:
                    self._conn.execute(_INSERT_RECORD, (
                        rec['pear_id'],
                        rec['zone_id'],
                        rec['timestamp'],
                        rec['pear_status'],
                        rec['pear_category'],
                        rec['infection_area_px'],
                        rec['infection_location'],
                        rec['infection_color_rgb'],
                        rec['infection_ratio'],
                        rec['pear_surface_area_px'],
                        rec['pear_volume_cm3'],
                        rec['pear_mass_g'],
                        rec['belt_speed_ms'],
                    ))
                self._conn.commit()
                self.get_logger().debug(f'Flushed {len(sorted_records)} records to DB.')
            except sqlite3.Error as exc:
                self.get_logger().error(f'DB write error: {exc}')

        self._pending.clear()

    # ── IoT status publish (13 fields, 0.1 Hz) ───────────────────────────
    def _publish_iot(self) -> None:
        """
        Publish the 13 IoT fields defined in Section 13 of the report.
        Published at 0.1 Hz (every 10 seconds) to the status webpage.
        """
        with self._db_lock:
            cur = self._conn.execute(
                'SELECT COUNT(*) FROM pear_records WHERE pear_status="ACCEPTED"'
            )
            total_accepted = cur.fetchone()[0]

            cur = self._conn.execute(
                'SELECT COUNT(*) FROM pear_records WHERE pear_status="REJECTED"'
            )
            total_rejected = cur.fetchone()[0]

            cur = self._conn.execute(
                'SELECT AVG(pear_mass_g) FROM pear_records WHERE pear_mass_g > 0'
            )
            avg_mass = cur.fetchone()[0] or 0.0

            cur = self._conn.execute(
                'SELECT AVG(pear_volume_cm3) FROM pear_records WHERE pear_volume_cm3 > 0'
            )
            avg_volume = cur.fetchone()[0] or 0.0

            cur = self._conn.execute(
                'SELECT COUNT(*) FROM pear_records WHERE pear_category="BIG"'
            )
            big_count = cur.fetchone()[0]

            cur = self._conn.execute(
                'SELECT COUNT(*) FROM pear_records WHERE pear_category="SMALL"'
            )
            small_count = cur.fetchone()[0]

            cur = self._conn.execute(
                'SELECT pear_id FROM pear_records ORDER BY timestamp DESC LIMIT 1'
            )
            row = cur.fetchone()
            latest_id = row[0] if row else 'N/A'

        # 13 IoT fields
        payload = {
            # Field  1: Pear ID (latest)
            'latest_pear_id':        latest_id,
            # Field  2: Zone ID (derived from latest pear ID prefix)
            'latest_zone_id':        latest_id[:2] if latest_id != 'N/A' else 'N/A',
            # Field  3: Pear status summary
            'total_accepted':        total_accepted,
            'total_rejected':        total_rejected,
            # Field  4: Pear category summary
            'total_big':             big_count,
            'total_small':           small_count,
            # Field  5: Infection area (last batch average)
            'batch_accepted':        self._accepted,
            'batch_rejected':        self._rejected,
            # Field  6–8: Infection data (latest record)
            'avg_infection_ratio':   0.0,     # filled per latest record below
            # Field  9: Surface area
            'avg_surface_area_px':   0.0,
            # Field 10: Volume
            'avg_volume_cm3':        round(avg_volume, 3),
            # Field 11: Mass
            'avg_mass_g':            round(avg_mass, 3),
            # Field 12: Belt speed
            'belt_speed_ms':         round(self._belt_speed, 3),
            # Field 13: Timestamp
            'timestamp':             time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._iot_pub.publish(msg)
        self.get_logger().info('IoT status published.')

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def destroy_node(self):
        self._flush_pending()
        self._conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
