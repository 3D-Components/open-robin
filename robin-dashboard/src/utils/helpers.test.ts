import { describe, expect, it, vi } from 'vitest';
import { clamp01, cx, gateForConfidence, jitter, reasonsFor, shortId, smooth } from './helpers';

describe('dashboard helper utilities', () => {
    it('joins truthy class names in order', () => {
        expect(cx('base', false, 'enabled', null, undefined, 'active')).toBe('base enabled active');
    });

    it('clamps values to the unit interval', () => {
        expect(clamp01(-0.2)).toBe(0);
        expect(clamp01(0.45)).toBe(0.45);
        expect(clamp01(1.5)).toBe(1);
    });

    it('smooths a value toward its target', () => {
        expect(smooth(10, 20, 0.25)).toBe(12.5);
    });

    it('maps confidence to trust gates', () => {
        expect(gateForConfidence(0.2, 0.7, 0.4)).toBe('Stop');
        expect(gateForConfidence(0.5, 0.7, 0.4)).toBe('Warning');
        expect(gateForConfidence(0.9, 0.7, 0.4)).toBe('OK');
    });

    it('deduplicates confidence reason labels', () => {
        expect(reasonsFor(0.3)).toEqual(['out-of-distribution', 'low confidence', 'sensor anomaly']);
        expect(reasonsFor(0.9)).toEqual(['nominal']);
    });

    it('uses Math.random consistently for generated demo values', () => {
        const randomSpy = vi.spyOn(Math, 'random').mockReturnValue(0.5);

        expect(shortId('RUN')).toBe('RUN-8');
        expect(jitter(10, 4)).toBe(10);

        randomSpy.mockRestore();
    });
});
