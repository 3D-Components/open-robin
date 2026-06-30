import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { Button } from './Button';

describe('Button', () => {
    it('renders primary buttons with default sizing', () => {
        render(<Button title="Start process">Start</Button>);

        const button = screen.getByRole('button', { name: 'Start' });
        expect(button).toHaveAttribute('title', 'Start process');
        expect(button).toHaveClass('h-9');
        expect(button).toHaveClass('bg-slate-900');
    });

    it('applies variant, size, and custom classes', () => {
        render(
            <Button variant="danger" size="lg" className="extra-class">
                Stop
            </Button>,
        );

        const button = screen.getByRole('button', { name: 'Stop' });
        expect(button).toHaveClass('h-10');
        expect(button).toHaveClass('bg-rose-600');
        expect(button).toHaveClass('extra-class');
    });

    it('fires clicks unless disabled', () => {
        const onClick = vi.fn();
        const { rerender } = render(<Button onClick={onClick}>Pause</Button>);

        fireEvent.click(screen.getByRole('button', { name: 'Pause' }));
        expect(onClick).toHaveBeenCalledTimes(1);

        rerender(
            <Button disabled onClick={onClick}>
                Pause
            </Button>,
        );
        fireEvent.click(screen.getByRole('button', { name: 'Pause' }));
        expect(onClick).toHaveBeenCalledTimes(1);
    });
});
