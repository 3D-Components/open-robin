import type { TrustGate } from '../types';

// Classname utility
export function cx(...classes: Array<string | undefined | false | null>) {
    return classes.filter(Boolean).join(" ");
}

// Time helpers
export function nowIso() {
    return new Date().toISOString();
}

export function shortId(prefix: string) {
    return `${prefix}-${Math.random().toString(16).slice(2, 8).toUpperCase()}`;
}

// Math helpers
export function clamp01(x: number) {
    return Math.max(0, Math.min(1, x));
}

export function jitter(base: number, amount: number) {
    return base + (Math.random() - 0.5) * 2 * amount;
}

export function smooth(prev: number, target: number, alpha: number) {
    return prev + (target - prev) * alpha;
}

// Trust logic
export function gateForConfidence(conf: number, warnTh: number, stopTh: number): TrustGate {
    if (conf < stopTh) return "Stop";
    if (conf < warnTh) return "Warning";
    return "OK";
}

export function reasonsFor(conf: number): string[] {
    const r: string[] = [];
    if (conf < 0.35) r.push("out-of-distribution");
    if (conf < 0.55) r.push("low confidence");
    if (conf < 0.45) r.push("sensor anomaly");
    if (r.length === 0) r.push("nominal");
    return Array.from(new Set(r));
}
