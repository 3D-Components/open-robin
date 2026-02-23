import { createContext, useContext, useEffect, useState, type ReactNode } from 'react';
import { domainTerms, applyProfileVocabulary, type DomainTerms } from './domain';

interface ProfileSkill {
    ros_type: string;
    ros_name: string;
    srv_type?: string;
    action_type?: string;
    description: string;
}

interface FieldSpec {
    label: string;
    unit: string;
}

export interface ProfileData {
    profile: { name: string; description: string };
    vocabulary: Partial<DomainTerms>;
    fields: Record<string, FieldSpec>;
    skills: Record<string, ProfileSkill>;
    ros2: Record<string, unknown>;
    ai: Record<string, unknown>;
    dds: Record<string, unknown>;
}

interface ProfileContextValue {
    terms: DomainTerms;
    data: ProfileData | null;
    loading: boolean;
}

const ProfileContext = createContext<ProfileContextValue>({
    terms: domainTerms,
    data: null,
    loading: true,
});

const API_URL: string =
    import.meta.env.VITE_ROBIN_API_URL
    ?? (import.meta.env.DEV ? '/api' : 'http://localhost:8001');

export function ProfileProvider({ children }: { children: ReactNode }) {
    const [data, setData] = useState<ProfileData | null>(null);
    const [loading, setLoading] = useState(true);
    const [, setTick] = useState(0);

    useEffect(() => {
        let cancelled = false;
        (async () => {
            try {
                const res = await fetch(`${API_URL}/profile`);
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                const profile: ProfileData = await res.json();
                if (cancelled) return;
                setData(profile);
                if (profile.vocabulary) {
                    applyProfileVocabulary(profile.vocabulary);
                    setTick((t) => t + 1);
                }
            } catch {
                // API unreachable - env-var / built-in defaults remain active
            } finally {
                if (!cancelled) setLoading(false);
            }
        })();
        return () => { cancelled = true; };
    }, []);

    return (
        <ProfileContext.Provider value={{ terms: domainTerms, data, loading }}>
            {children}
        </ProfileContext.Provider>
    );
}

export function useProfile(): ProfileContextValue {
    return useContext(ProfileContext);
}

export function useTerms(): DomainTerms {
    return useContext(ProfileContext).terms;
}
