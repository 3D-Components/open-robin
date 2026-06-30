import type { ProfileData } from './ProfileContext';
import type { AIInputFeatureSpec, AIInputParams } from '../types';

function numberOrUndefined(value: unknown): number | undefined {
    return typeof value === 'number' && Number.isFinite(value) ? value : undefined;
}

function buildDefaultFallback(): AIInputFeatureSpec[] {
    return [
        {
            key: 'wire_feed_speed_mpm_model_input',
            label: 'Wire Feed Speed',
            unit: 'm/min',
            defaultValue: 10,
            step: 0.1,
        },
        {
            key: 'travel_speed_mps_model_input',
            label: 'Travel Speed',
            unit: 'm/s',
            defaultValue: 0.02,
            step: 0.001,
        },
        {
            key: 'arc_length_correction_mm_model_input',
            label: 'Arc Length Correction',
            unit: 'mm',
            defaultValue: 0,
            step: 0.1,
        },
    ];
}

export function resolveAIInputFeatures(profile: ProfileData | null): AIInputFeatureSpec[] {
    const raw = profile?.ai?.input_features;
    if (!Array.isArray(raw) || raw.length === 0) {
        return buildDefaultFallback();
    }

    const specs: AIInputFeatureSpec[] = [];
    for (const item of raw) {
        if (!item || typeof item !== 'object') continue;
        const rawItem = item as Record<string, unknown>;
        const key = typeof rawItem.key === 'string' ? rawItem.key : '';
        const label = typeof rawItem.label === 'string' ? rawItem.label : key;
        if (!key || !label) continue;
        const aliases = Array.isArray(rawItem.aliases)
            ? rawItem.aliases.filter((alias): alias is string => typeof alias === 'string' && alias.length > 0)
            : [];
        specs.push({
            key,
            label,
            unit: typeof rawItem.unit === 'string' ? rawItem.unit : '',
            aliases: [key, ...aliases.filter((alias) => alias !== key)],
            defaultValue: numberOrUndefined(rawItem.default),
            step: numberOrUndefined(rawItem.step),
            min: numberOrUndefined(rawItem.min),
            max: numberOrUndefined(rawItem.max),
        });
    }

    return specs.length > 0 ? specs : buildDefaultFallback();
}

export function buildDefaultAIInputParams(specs: AIInputFeatureSpec[]): AIInputParams {
    const params: AIInputParams = {};
    for (const spec of specs) {
        params[spec.key] = spec.defaultValue ?? 0;
    }
    return params;
}

export function normalizeAIInputParams(
    params: AIInputParams | null | undefined,
    specs: AIInputFeatureSpec[],
): AIInputParams {
    const next = buildDefaultAIInputParams(specs);
    if (!params) return next;

    for (const spec of specs) {
        const aliases = spec.aliases ?? [spec.key];
        for (const alias of aliases) {
            const value = params[alias];
            if (typeof value === 'number' && Number.isFinite(value)) {
                next[spec.key] = value;
                break;
            }
        }
    }

    return next;
}

export function mergeRecommendedAIInputParams(
    recommended: Record<string, unknown> | undefined,
    fallback: AIInputParams,
    specs: AIInputFeatureSpec[],
): AIInputParams {
    const next = normalizeAIInputParams(fallback, specs);
    if (!recommended) return next;

    for (const spec of specs) {
        const aliases = spec.aliases ?? [spec.key];
        for (const alias of aliases) {
            const value = recommended[alias];
            if (typeof value === 'number' && Number.isFinite(value)) {
                next[spec.key] = value;
                break;
            }
        }
    }

    return next;
}

export function resolveRecordedAIInputParams(
    sample: Record<string, unknown> | null | undefined,
    specs: AIInputFeatureSpec[],
): AIInputParams {
    const rawNested = sample?.input_params;
    if (rawNested && typeof rawNested === 'object') {
        return normalizeAIInputParams(rawNested as AIInputParams, specs);
    }
    return normalizeAIInputParams(sample as AIInputParams | null | undefined, specs);
}

export function formatAIInputSummary(
    params: AIInputParams,
    specs: AIInputFeatureSpec[],
): string {
    return specs
        .map((spec) => {
            const value = params[spec.key] ?? 0;
            const decimals = spec.step && spec.step < 1 ? 3 : 2;
            const suffix = spec.unit ? ` ${spec.unit}` : '';
            return `${spec.label}=${value.toFixed(decimals)}${suffix}`;
        })
        .join(', ');
}
