#!/usr/bin/env python3
"""
WeldingHttpBridgeNode: HTTP → /intents bridge for the ROBIN React dashboard.

Runs a lightweight aiohttp HTTP server on port 8766 alongside a ROS2 node.

## Intent delivery paths

### Direct HTTP (for testing / fallback)
    POST /intent
        Body (JSON):  {"intent": "<INTENT_CONSTANT>", "data": {...}}
        Response 200: {"status": "published", "intent": "<INTENT_CONSTANT>"}
        Response 400: {"error": "<message>"}

### Via Orion-LD (production path — Orion-LD as the central broker)
    The bridge registers an NGSI-LD subscription at startup:
      - watches pendingIntent changes on urn:ngsi-ld:Process:* entities
      - Orion-LD POSTs notifications to POST /orion-notify

    POST /orion-notify
        Body: NGSI-LD notification (application/json)
        Extracts pendingIntent.value.{intent, data} and publishes to /intents

### Other
    GET /health
        Response 200: {"status": "ok"}

CORS is enabled for all origins so the dashboard can call it from
http://localhost:5174 (Vite dev server) or any production origin.

Architecture mirrors IntentBridgeNode from welding_gui — rclpy
Publisher.publish() is thread-safe so the aiohttp event loop can call it
directly from its async handler without additional locking.
"""
import json
import os
import threading
import time
import urllib.error
import urllib.request

import rclpy
from aiohttp import web
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from welding_msgs.msg import Intent

PORT = 8766
ORION_URL = os.getenv('ORION_URL', 'http://localhost:1026')
NOTIFY_URL = os.getenv('NOTIFY_URL', f'http://localhost:{PORT}/orion-notify')
SUBSCRIPTION_ID = 'urn:ngsi-ld:Subscription:welding-intent-bridge'


class WeldingHttpBridgeNode(Node):
    """ROS2 node + aiohttp HTTP server on port 8766."""

    TOPIC = '/intents'

    def __init__(self):
        super().__init__('welding_http_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._intent_pub = self.create_publisher(Intent, self.TOPIC, qos)
        self.get_logger().info(
            f'WeldingHttpBridgeNode ready — publishing on {self.TOPIC}, '
            f'HTTP server on port {PORT}'
        )

    def publish_intent(self, intent_type: str, data: dict) -> None:
        """Thread-safe intent publisher (called from aiohttp async handlers)."""
        msg = Intent()
        msg.intent     = intent_type
        msg.data       = json.dumps(data)
        msg.source     = Intent.SOURCE_REMOTE
        msg.modality   = Intent.MODALITY_TOUCHSCREEN
        msg.priority   = 0.5
        msg.confidence = 1.0
        self._intent_pub.publish(msg)
        self.get_logger().info(
            f'Published intent: {intent_type} | data: {msg.data}'
        )


# ── Orion-LD subscription registration ────────────────────────────────────────

def _register_subscription_background() -> None:
    """Keep the NGSI-LD subscription alive.

    Polls every 30 s. Deletes and re-registers if missing or paused.
    Never exits — runs as a daemon thread for the lifetime of the bridge.
    """
    sub_body = json.dumps({
        'id': SUBSCRIPTION_ID,
        'type': 'Subscription',
        'entities': [{'idPattern': 'urn:ngsi-ld:Process:.*', 'type': 'urn:robin:Process'}],
        'watchedAttributes': ['pendingIntent'],
        'notification': {
            'format': 'normalized',
            'endpoint': {'uri': NOTIFY_URL, 'accept': 'application/json'},
        },
    }).encode('utf-8')

    def _is_active() -> bool:
        req = urllib.request.Request(
            f'{ORION_URL}/ngsi-ld/v1/subscriptions/{SUBSCRIPTION_ID}',
            headers={'Accept': 'application/json'},
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read())
            return data.get('isActive', True) and data.get('status') == 'active'

    def _delete():
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    f'{ORION_URL}/ngsi-ld/v1/subscriptions/{SUBSCRIPTION_ID}',
                    method='DELETE',
                ),
                timeout=5,
            )
        except Exception:
            pass

    def _register():
        urllib.request.urlopen(
            urllib.request.Request(
                f'{ORION_URL}/ngsi-ld/v1/subscriptions',
                data=sub_body,
                headers={'Content-Type': 'application/json'},
                method='POST',
            ),
            timeout=5,
        )

    while True:
        try:
            if _is_active():
                time.sleep(30)
                continue
            # Missing (HTTPError 404) or paused — delete stale copy and re-register
        except urllib.error.HTTPError as e:
            if e.code != 404:
                time.sleep(5)
                continue
        except Exception:
            time.sleep(5)
            continue

        try:
            _delete()
            time.sleep(1)
            _register()
        except Exception:
            pass
        time.sleep(5)


# ── aiohttp request handlers ───────────────────────────────────────────────

def _cors_headers() -> dict:
    return {
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Methods': 'POST, GET, OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type',
    }


def make_app(ros_node: WeldingHttpBridgeNode) -> web.Application:
    app = web.Application()

    async def handle_health(request: web.Request) -> web.Response:
        return web.Response(
            text=json.dumps({'status': 'ok'}),
            content_type='application/json',
            headers=_cors_headers(),
        )

    async def handle_options(request: web.Request) -> web.Response:
        # Pre-flight CORS response
        return web.Response(status=204, headers=_cors_headers())

    async def handle_intent(request: web.Request) -> web.Response:
        try:
            body = await request.json()
        except Exception:
            return web.Response(
                status=400,
                text=json.dumps({'error': 'Request body must be valid JSON'}),
                content_type='application/json',
                headers=_cors_headers(),
            )

        intent_type = body.get('intent', '').strip()
        data        = body.get('data', {})

        if not intent_type:
            return web.Response(
                status=400,
                text=json.dumps({'error': 'Missing required field: intent'}),
                content_type='application/json',
                headers=_cors_headers(),
            )

        if not isinstance(data, dict):
            return web.Response(
                status=400,
                text=json.dumps({'error': 'Field "data" must be a JSON object'}),
                content_type='application/json',
                headers=_cors_headers(),
            )

        ros_node.publish_intent(intent_type, data)
        return web.Response(
            text=json.dumps({'status': 'published', 'intent': intent_type}),
            content_type='application/json',
            headers=_cors_headers(),
        )

    async def handle_orion_notify(request: web.Request) -> web.Response:
        """Receive NGSI-LD subscription notification from Orion-LD.

        Orion-LD sends this when the pendingIntent attribute changes on a
        Process entity. Extracts the intent and publishes to /intents.

        Expected body structure:
          {
            "data": [
              {
                "id": "urn:ngsi-ld:Process:ros_bridge",
                "pendingIntent": {
                  "type": "Property",
                  "value": {"intent": "START_PROCESS", "data": {...}}
                }
              }
            ]
          }
        """
        try:
            body = await request.json()
        except Exception:
            return web.Response(
                status=400,
                text=json.dumps({'error': 'Invalid JSON'}),
                content_type='application/json',
            )

        for entity in body.get('data', []):
            value = entity.get('pendingIntent', {}).get('value', {})
            intent_type = value.get('intent', '').strip()
            data = value.get('data', {})
            if intent_type:
                ros_node.publish_intent(intent_type, data if isinstance(data, dict) else {})

        return web.Response(
            status=200,
            text=json.dumps({'status': 'ok'}),
            content_type='application/json',
        )

    app.router.add_get('/health', handle_health)
    app.router.add_options('/intent', handle_options)
    app.router.add_post('/intent', handle_intent)
    app.router.add_post('/orion-notify', handle_orion_notify)
    return app


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WeldingHttpBridgeNode()

    # Spin rclpy in a background thread so aiohttp can run in the main event loop
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    ros_thread.start()

    # Register the Orion-LD subscription in a separate daemon thread
    # (retries until Orion-LD is available, without blocking HTTP server startup)
    sub_thread = threading.Thread(target=_register_subscription_background, daemon=True)
    sub_thread.start()

    app = make_app(node)

    try:
        web.run_app(app, host='0.0.0.0', port=PORT)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
