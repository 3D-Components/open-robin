// Minimal k6 smoke test for the Alert Engine API
// Usage: k6 run load/k6_api_smoke.js --env BASE_URL=http://localhost:8000

import http from 'k6/http';
import { check, sleep } from 'k6';

export const options = {
  vus: 5,
  duration: '30s',
};

const BASE = __ENV.BASE_URL || 'http://localhost:8000';

export default function () {
  const resRoot = http.get(`${BASE}/`);
  check(resRoot, {
    'root 200': (r) => r.status === 200,
  });

  const resHealth = http.get(`${BASE}/health`);
  check(resHealth, {
    'health 200': (r) => r.status === 200,
  });

  const resPredict = http.post(`${BASE}/ai/models/predict`, JSON.stringify({
    wireSpeed: 2.5,
    current: 110,
    voltage: 17,
  }), { headers: { 'Content-Type': 'application/json' }});
  check(resPredict, {
    'predict 200': (r) => r.status === 200,
  });

  sleep(1);
}

