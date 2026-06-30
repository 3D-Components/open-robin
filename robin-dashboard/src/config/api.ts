export const ROBIN_API_URL: string =
    import.meta.env.VITE_ROBIN_API_URL
    ?? (import.meta.env.DEV ? '/api' : 'http://localhost:8000');
