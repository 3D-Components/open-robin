import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { Divider, KV, ProgressBar } from './ProgressBar';

describe('ProgressBar UI primitives', () => {
    it('clamps rendered progress width between 0 and 100 percent', () => {
        const { container, rerender } = render(<ProgressBar value={125} tone="good" />);
        const fill = container.querySelector('.bg-emerald-600') as HTMLElement;

        expect(fill).toHaveStyle({ width: '100%' });

        rerender(<ProgressBar value={-4} tone="bad" />);
        const badFill = container.querySelector('.bg-rose-600') as HTMLElement;
        expect(badFill).toHaveStyle({ width: '0%' });
    });

    it('renders key-value labels with string titles for truncated values', () => {
        render(<KV k="Process" v="run-001" mono />);

        expect(screen.getByText('Process')).toBeInTheDocument();
        expect(screen.getByText('run-001')).toHaveAttribute('title', 'run-001');
        expect(screen.getByText('run-001')).toHaveClass('font-mono');
    });

    it('renders a divider element', () => {
        const { container } = render(<Divider />);

        expect(container.firstElementChild).toHaveClass('h-px');
    });
});
