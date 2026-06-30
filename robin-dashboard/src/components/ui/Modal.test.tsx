import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import { Modal } from './Modal';

describe('Modal', () => {
    it('renders nothing when closed', () => {
        const { container } = render(
            <Modal open={false} title="Confirm" onConfirm={vi.fn()} onClose={vi.fn()}>
                Hidden
            </Modal>,
        );

        expect(container).toBeEmptyDOMElement();
    });

    it('renders content, danger hint, and handles actions', () => {
        const onClose = vi.fn();
        const onConfirm = vi.fn();

        render(
            <Modal
                open
                title="Stop process"
                dangerHint="This pauses the run"
                confirmText="Stop"
                confirmVariant="danger"
                onClose={onClose}
                onConfirm={onConfirm}
            >
                Confirm operator action
            </Modal>,
        );

        expect(screen.getByText('Stop process')).toBeInTheDocument();
        expect(screen.getByText('This pauses the run')).toBeInTheDocument();
        expect(screen.getByText('Confirm operator action')).toBeInTheDocument();

        fireEvent.click(screen.getByRole('button', { name: 'Close' }));
        fireEvent.click(screen.getByRole('button', { name: 'Cancel' }));
        fireEvent.click(screen.getByRole('button', { name: 'Stop' }));

        expect(onClose).toHaveBeenCalledTimes(2);
        expect(onConfirm).toHaveBeenCalledTimes(1);
    });

    it('disables confirm when requested', () => {
        render(
            <Modal open title="Confirm" confirmDisabled onClose={vi.fn()} onConfirm={vi.fn()}>
                Body
            </Modal>,
        );

        expect(screen.getByRole('button', { name: 'Confirm' })).toBeDisabled();
    });
});
