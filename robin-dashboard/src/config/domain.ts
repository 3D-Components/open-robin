const env = import.meta.env as Record<string, string | undefined>;

function term(name: string, fallback: string): string {
    const value = env[name];
    if (!value) return fallback;
    const cleaned = value.trim();
    return cleaned.length > 0 ? cleaned : fallback;
}

const _defaults = {
    process: term('VITE_TERM_PROCESS', 'Process'),
    processPlural: term('VITE_TERM_PROCESS_PLURAL', 'Processes'),
    segment: term('VITE_TERM_SEGMENT', 'Segment'),
    segmentPlural: term('VITE_TERM_SEGMENT_PLURAL', 'Segments'),
    geometry: term('VITE_TERM_GEOMETRY', 'Geometry'),
    profileHeight: term('VITE_TERM_PROFILE_HEIGHT', 'Profile Height'),
    profileWidth: term('VITE_TERM_PROFILE_WIDTH', 'Profile Width'),
    depositionView: term('VITE_TERM_DEPOSITION_VIEW', 'Deposition'),
    speed: term('VITE_TERM_SPEED', 'Speed'),
    speedUnit: term('VITE_TERM_SPEED_UNIT', 'mm/s'),
    current: term('VITE_TERM_CURRENT', 'Current'),
    currentUnit: term('VITE_TERM_CURRENT_UNIT', 'A'),
    voltage: term('VITE_TERM_VOLTAGE', 'Voltage'),
    voltageUnit: term('VITE_TERM_VOLTAGE_UNIT', 'V'),
    toolPath: term('VITE_TERM_TOOL_PATH', 'Tool path'),
    workpiece: term('VITE_TERM_WORKPIECE', 'Workpiece'),
};

export type DomainTerms = typeof _defaults;

/**
 * Mutable vocabulary object.  At import time it contains env-var / built-in
 * defaults.  When the ProfileContext fetches the profile YAML from the API it
 * calls {@link applyProfileVocabulary} to patch the values in-place so that
 * module-level references (e.g. METRIC_LABELS) pick up the runtime config on
 * next render.
 */
export const domainTerms: DomainTerms = { ..._defaults };

export function applyProfileVocabulary(vocab: Partial<DomainTerms>): void {
    Object.assign(domainTerms, vocab);
}
