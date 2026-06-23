import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { Toggle } from './Toggle';

describe('Toggle', () => {
    it('renders label and hint, then emits the next checked state', () => {
        const onChange = vi.fn();

        render(<Toggle checked={false} onChange={onChange} label="Mock data" hint="Use demo telemetry" />);
        fireEvent.click(screen.getByRole('button', { name: /mock data/i }));

        expect(screen.getByText('Use demo telemetry')).toBeInTheDocument();
        expect(onChange).toHaveBeenCalledWith(true);
    });

    it('emits false when currently checked', () => {
        const onChange = vi.fn();

        render(<Toggle checked onChange={onChange} label="Live mode" />);
        fireEvent.click(screen.getByRole('button', { name: /live mode/i }));

        expect(onChange).toHaveBeenCalledWith(false);
    });
});
