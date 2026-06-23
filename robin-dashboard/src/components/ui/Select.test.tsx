import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { Select } from './Select';

describe('Select', () => {
    it('renders options and emits selected values', () => {
        const onChange = vi.fn();

        render(
            <Select
                value="demo"
                className="custom-select"
                onChange={onChange}
                options={[
                    { value: 'demo', label: 'Demo' },
                    { value: 'live', label: 'Live' },
                ]}
            />,
        );

        const select = screen.getByRole('combobox');
        expect(select.parentElement).toHaveClass('custom-select');

        fireEvent.change(select, { target: { value: 'live' } });

        expect(onChange).toHaveBeenCalledWith('live');
    });
});
