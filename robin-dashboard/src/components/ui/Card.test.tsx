import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { Card, CardBody, CardHeader } from './Card';

describe('Card primitives', () => {
    it('renders card content with optional custom classes', () => {
        render(<Card className="custom-card">Body</Card>);

        expect(screen.getByText('Body')).toHaveClass('custom-card');
    });

    it('renders header title, subtitle, and right slot', () => {
        render(<CardHeader title="Run" subtitle="Active" right={<button type="button">Menu</button>} />);

        expect(screen.getByText('Run')).toBeInTheDocument();
        expect(screen.getByText('Active')).toBeInTheDocument();
        expect(screen.getByRole('button', { name: 'Menu' })).toBeInTheDocument();
    });

    it('omits optional header slots when not provided', () => {
        render(<CardHeader title={<span>Minimal</span>} />);

        expect(screen.getByText('Minimal')).toBeInTheDocument();
    });

    it('renders card body content with optional classes', () => {
        render(<CardBody className="compact">Controls</CardBody>);

        expect(screen.getByText('Controls')).toHaveClass('compact');
    });
});
