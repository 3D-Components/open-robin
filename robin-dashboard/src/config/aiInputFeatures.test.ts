import { describe, expect, it } from 'vitest';
import {
    buildDefaultAIInputParams,
    formatAIInputSummary,
    mergeRecommendedAIInputParams,
    normalizeAIInputParams,
    resolveAIInputFeatures,
    resolveRecordedAIInputParams,
} from './aiInputFeatures';
import type { AIInputFeatureSpec } from '../types';
import type { ProfileData } from './ProfileContext';

const coatingProfile = {
    ai: {
        input_features: [
            {
                key: 'flow_rate',
                label: 'Flow Rate',
                unit: 'L/min',
                aliases: ['current'],
                default: 8.5,
                step: 0.1,
                min: 0,
                max: 20,
            },
            {
                key: 'line_speed',
                label: 'Line Speed',
                unit: 'm/s',
                aliases: ['travel_speed'],
                default: 0.12,
                step: 0.001,
            },
            {
                key: '',
                label: 'Ignored',
            },
        ],
    },
} as unknown as ProfileData;

describe('AI input feature configuration', () => {
    it('resolves profile feature specs and keeps key aliases stable', () => {
        const specs = resolveAIInputFeatures(coatingProfile);

        expect(specs).toHaveLength(2);
        expect(specs[0]).toMatchObject({
            key: 'flow_rate',
            label: 'Flow Rate',
            unit: 'L/min',
            aliases: ['flow_rate', 'current'],
            defaultValue: 8.5,
            step: 0.1,
            min: 0,
            max: 20,
        });
    });

    it('falls back to the welding-compatible default feature contract', () => {
        const specs = resolveAIInputFeatures(null);

        expect(specs.map((spec) => spec.key)).toEqual([
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ]);
    });

    it('builds and normalizes params from defaults, aliases, and finite values', () => {
        const specs = resolveAIInputFeatures(coatingProfile);

        expect(buildDefaultAIInputParams(specs)).toEqual({
            flow_rate: 8.5,
            line_speed: 0.12,
        });
        expect(
            normalizeAIInputParams(
                {
                    current: 9.1,
                    travel_speed: 0.18,
                    flow_rate: Number.NaN,
                },
                specs,
            ),
        ).toEqual({
            flow_rate: 9.1,
            line_speed: 0.18,
        });
    });

    it('merges recommendations over fallback values using aliases', () => {
        const specs = resolveAIInputFeatures(coatingProfile);

        expect(
            mergeRecommendedAIInputParams(
                { current: 10.4 },
                { flow_rate: 8.5, line_speed: 0.12 },
                specs,
            ),
        ).toEqual({
            flow_rate: 10.4,
            line_speed: 0.12,
        });
    });

    it('resolves recorded sample params from nested or flat sample records', () => {
        const specs = resolveAIInputFeatures(coatingProfile);

        expect(resolveRecordedAIInputParams({ input_params: { current: 7.2 } }, specs)).toEqual({
            flow_rate: 7.2,
            line_speed: 0.12,
        });
        expect(resolveRecordedAIInputParams({ travel_speed: 0.2 }, specs)).toEqual({
            flow_rate: 8.5,
            line_speed: 0.2,
        });
    });

    it('formats a compact operator-facing summary', () => {
        const specs: AIInputFeatureSpec[] = [
            { key: 'flow_rate', label: 'Flow Rate', unit: 'L/min', step: 0.1 },
            { key: 'line_speed', label: 'Line Speed', unit: 'm/s', step: 0.001 },
        ];

        expect(formatAIInputSummary({ flow_rate: 8.5, line_speed: 0.1234 }, specs)).toBe(
            'Flow Rate=8.500 L/min, Line Speed=0.123 m/s',
        );
    });
});
