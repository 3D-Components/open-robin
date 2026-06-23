import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { Chip } from './Chip';

describe('Chip', () => {
    it.each([
        ['neutral', 'bg-slate-100'],
        ['good', 'bg-emerald-50'],
        ['warn', 'bg-amber-50'],
        ['bad', 'bg-rose-50'],
        ['info', 'bg-cyan-50'],
        ['ghost', 'bg-transparent'],
    ] as const)('renders the %s tone', (tone, expectedClass) => {
        render(
            <Chip tone={tone} className="custom-chip">
                {tone}
            </Chip>,
        );

        const chip = screen.getByText(tone);
        expect(chip).toHaveClass(expectedClass);
        expect(chip).toHaveClass('custom-chip');
    });
});
