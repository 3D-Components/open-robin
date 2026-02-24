import { useState } from 'react';
import { RefreshCw, ExternalLink, AlertTriangle } from 'lucide-react';
import { Button } from '../../ui/Button';
import { Chip } from '../../ui/Chip';

interface ViserViewerProps {
    /** URL of the Viser server (default: http://localhost:8081) */
    serverUrl?: string;
    /** Height of the viewer */
    height?: string;
    /** Whether to show the connection status bar */
    showStatus?: boolean;
}

/**
 * Embeds a Viser 3D visualization server into the React app via iframe.
 * Requires the Viser server to be running separately.
 */
export function ViserViewer({
    serverUrl = 'http://localhost:8081',
    height = '500px',
    showStatus = true,
}: ViserViewerProps) {
    const [connected, setConnected] = useState(true);
    const [key, setKey] = useState(0);

    const handleRefresh = () => {
        setKey((k) => k + 1);
        setConnected(true);
    };

    const handleError = () => {
        setConnected(false);
    };

    return (
        <div className="flex flex-col rounded-xl border border-slate-200 bg-white overflow-hidden dark:border-slate-800 dark:bg-slate-950">
            {showStatus && (
                <div className="flex items-center justify-between gap-3 border-b border-slate-200 px-3 py-2 dark:border-slate-800">
                    <div className="flex items-center gap-2">
                        <Chip tone={connected ? 'good' : 'bad'}>
                            {connected ? 'Connected' : 'Disconnected'}
                        </Chip>
                        <span className="text-xs text-slate-600 dark:text-slate-400">
                            {serverUrl}
                        </span>
                    </div>
                    <div className="flex items-center gap-2">
                        <Button
                            size="sm"
                            variant="ghost"
                            onClick={handleRefresh}
                            title="Refresh viewer"
                        >
                            <RefreshCw className="h-4 w-4" />
                        </Button>
                        <Button
                            size="sm"
                            variant="ghost"
                            onClick={() => window.open(serverUrl, '_blank')}
                            title="Open in new tab"
                        >
                            <ExternalLink className="h-4 w-4" />
                        </Button>
                    </div>
                </div>
            )}

            <div className="relative" style={{ height }}>
                {connected ? (
                    <iframe
                        key={key}
                        src={serverUrl}
                        className="w-full h-full border-0"
                        title="Viser 3D Viewer"
                        onError={handleError}
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope"
                    />
                ) : (
                    <div className="absolute inset-0 flex items-center justify-center bg-slate-50 dark:bg-slate-900/50">
                        <div className="text-center max-w-md p-6">
                            <div className="mx-auto grid h-12 w-12 place-items-center rounded-xl border border-amber-300 bg-amber-50 dark:border-amber-700 dark:bg-amber-900/30">
                                <AlertTriangle className="h-6 w-6 text-amber-600 dark:text-amber-400" />
                            </div>
                            <div className="mt-3 text-sm font-semibold text-slate-900 dark:text-slate-100">
                                Viser Server Not Connected
                            </div>
                            <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                Make sure the Viser server is running:
                            </div>
                            <pre className="mt-2 rounded-lg bg-slate-900 p-2 text-xs text-slate-100 text-left overflow-x-auto">
                                cd visual_page{'\n'}
                                python viser_server.py
                            </pre>
                            <Button
                                size="sm"
                                className="mt-3"
                                onClick={handleRefresh}
                            >
                                <RefreshCw className="h-4 w-4" />
                                Retry Connection
                            </Button>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}
