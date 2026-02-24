import { useMemo, useState } from 'react';
import { FlaskConical, ShieldCheck, AlertTriangle, Server } from 'lucide-react';
import { Card, CardBody, CardHeader } from '../../ui/Card';
import { Button } from '../../ui/Button';
import { Chip } from '../../ui/Chip';
import { Divider, KV } from '../../ui/ProgressBar';

type HealthResponse = {
  ok: boolean;
  model_available: boolean;
  loaded: boolean;
  workspace_root: string;
  model_path: string;
  state_path: string;
  detail?: string;
};

type ConfigResponse = {
  ok: boolean;
  workspace_root: string;
  model_path: string;
  state_path: string;
  model_available: boolean;
  detail?: string;
};

type PredictResponse = {
  ok: boolean;
  target_dim: number;
  sample_count: number;
  lower: number[][];
  upper: number[][];
  intervals: number[];
  widths: number[][];
  midpoint: number[][];
};

const DEFAULT_INPUT = JSON.stringify(
  [[0.7, 600.0, 300.0, 150.0, 7.0, 3.0, 0.1, 3.0]],
  null,
  2
);

export function InferenceDevLabTab() {
  const [apiBase, setApiBase] = useState(
    import.meta.env.VITE_MCCP_API_URL || 'http://localhost:8091'
  );
  const [featuresText, setFeaturesText] = useState(DEFAULT_INPUT);
  const [targetDim, setTargetDim] = useState(2);
  const [recheckArtifacts, setRecheckArtifacts] = useState(true);
  const [workspaceRoot, setWorkspaceRoot] = useState('');
  const [workspaceEdited, setWorkspaceEdited] = useState(false);
  const [savingConfig, setSavingConfig] = useState(false);
  const [configMsg, setConfigMsg] = useState<string | null>(null);
  const [loadingHealth, setLoadingHealth] = useState(false);
  const [loadingPredict, setLoadingPredict] = useState(false);
  const [health, setHealth] = useState<HealthResponse | null>(null);
  const [prediction, setPrediction] = useState<PredictResponse | null>(null);
  const [error, setError] = useState<string | null>(null);

  const parsedInput = useMemo(() => {
    try {
      const parsed = JSON.parse(featuresText);
      if (!Array.isArray(parsed) || parsed.length === 0) {
        return { ok: false, message: 'Input must be a non-empty 2D numeric array.' };
      }
      const firstLen = Array.isArray(parsed[0]) ? parsed[0].length : -1;
      if (firstLen <= 0) {
        return { ok: false, message: 'Input must be a 2D array (n_samples x n_features).' };
      }
      for (const row of parsed) {
        if (!Array.isArray(row) || row.length !== firstLen) {
          return { ok: false, message: 'All rows must have the same feature count.' };
        }
        for (const value of row) {
          if (typeof value !== 'number' || Number.isNaN(value)) {
            return { ok: false, message: 'All feature values must be valid numbers.' };
          }
        }
      }
      return { ok: true, value: parsed as number[][] };
    } catch {
      return { ok: false, message: 'Input JSON is invalid.' };
    }
  }, [featuresText]);

  const fetchHealth = async () => {
    setLoadingHealth(true);
    setError(null);
    try {
      const res = await fetch(`${apiBase.replace(/\/$/, '')}/health`);
      if (!res.ok) throw new Error(`Health check failed: ${res.status}`);
      const data = (await res.json()) as HealthResponse;
      setHealth(data);
      // Sync local workspace root from server (only if user hasn't edited it)
      if (!workspaceEdited) {
        setWorkspaceRoot(data.workspace_root);
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to check inference API health.');
    } finally {
      setLoadingHealth(false);
    }
  };

  const saveConfig = async () => {
    if (!workspaceRoot.trim()) {
      setConfigMsg('Workspace root cannot be empty.');
      return;
    }
    setSavingConfig(true);
    setConfigMsg(null);
    setError(null);
    try {
      const res = await fetch(`${apiBase.replace(/\/$/, '')}/config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ workspace_root: workspaceRoot.trim() }),
      });
      const data = (await res.json()) as ConfigResponse;
      if (!res.ok || !data.ok) {
        throw new Error(data.detail || `Config update failed: ${res.status}`);
      }
      setConfigMsg(data.detail ?? 'Workspace updated.');
      setWorkspaceEdited(false);
      // Refresh health to reflect the new paths
      setHealth((prev) =>
        prev
          ? {
              ...prev,
              workspace_root: data.workspace_root,
              model_path: data.model_path,
              state_path: data.state_path,
              model_available: data.model_available,
              loaded: false, // model was unloaded on path change
            }
          : null
      );
      setWorkspaceRoot(data.workspace_root);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to update config.');
    } finally {
      setSavingConfig(false);
    }
  };

  const runInference = async () => {
    if (!parsedInput.ok) {
      setError(parsedInput.message ?? 'Input payload is invalid.');
      return;
    }
    setLoadingPredict(true);
    setError(null);
    try {
      const res = await fetch(`${apiBase.replace(/\/$/, '')}/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          input_features: parsedInput.value,
          target_dim: targetDim,
          reload_if_stale: recheckArtifacts,
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.ok) {
        throw new Error(data.detail || `Inference failed: ${res.status}`);
      }
      setPrediction(data as PredictResponse);
    } catch (e) {
      setError(e instanceof Error ? e.message : 'Failed to run inference.');
    } finally {
      setLoadingPredict(false);
    }
  };

  return (
    <div className="grid grid-cols-12 gap-4">
      <div className="col-span-12 xl:col-span-8 space-y-4">
        <Card>
          <CardHeader
            title={
              <span className="flex items-center gap-2">
                <FlaskConical className="h-4 w-4" />
                Inference DevLab
              </span>
            }
            subtitle="Development-only MCCP-UQ probe to validate model loading and uncertainty intervals."
            right={<Chip tone="warn">Dev only</Chip>}
          />
          <CardBody className="space-y-3">
            <div className="grid gap-3 md:grid-cols-2">
              <label className="text-xs text-slate-700 dark:text-slate-300">
                MCCP API base URL
                <input
                  className="mt-1 h-9 w-full rounded-md border border-slate-300 bg-white px-3 text-sm dark:border-slate-700 dark:bg-slate-950"
                  value={apiBase}
                  onChange={(e) => setApiBase(e.target.value)}
                />
              </label>
              <label className="text-xs text-slate-700 dark:text-slate-300">
                Target dimensions
                <input
                  type="number"
                  min={1}
                  max={8}
                  className="mt-1 h-9 w-full rounded-md border border-slate-300 bg-white px-3 text-sm dark:border-slate-700 dark:bg-slate-950"
                  value={targetDim}
                  onChange={(e) => setTargetDim(Math.max(1, Number(e.target.value) || 1))}
                />
              </label>
            </div>

            <label className="text-xs text-slate-700 dark:text-slate-300">
              Input features JSON (n_samples x n_features)
              <textarea
                className="mt-1 min-h-[140px] w-full rounded-md border border-slate-300 bg-white p-3 font-mono text-xs dark:border-slate-700 dark:bg-slate-950"
                value={featuresText}
                onChange={(e) => setFeaturesText(e.target.value)}
              />
            </label>

            <label className="inline-flex items-center gap-2 text-xs text-slate-700 dark:text-slate-300">
              <input
                type="checkbox"
                checked={recheckArtifacts}
                onChange={(e) => setRecheckArtifacts(e.target.checked)}
              />
              Reload model/state automatically when artifact files change
            </label>

            {!parsedInput.ok && <Chip tone="bad">{parsedInput.message}</Chip>}
            {error && <Chip tone="bad">{error}</Chip>}

            <div className="flex flex-wrap gap-2">
              <Button variant="secondary" onClick={fetchHealth} disabled={loadingHealth}>
                <Server className="h-4 w-4" />
                {loadingHealth ? 'Checking...' : 'Check API Health'}
              </Button>
              <Button onClick={runInference} disabled={loadingPredict || !parsedInput.ok}>
                <ShieldCheck className="h-4 w-4" />
                {loadingPredict ? 'Running...' : 'Run MCCP Inference'}
              </Button>
            </div>
          </CardBody>
        </Card>

        {prediction && (
          <Card>
            <CardHeader
              title="Prediction Results"
              subtitle="Conformal lower/upper bounds and uncertainty widths from MCCP-UQ."
            />
            <CardBody className="space-y-3">
              {prediction.lower.map((lowerRow, sampleIdx) => (
                <div
                  key={`sample-${sampleIdx}`}
                  className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950"
                >
                  <div className="mb-2 text-xs font-semibold">Sample {sampleIdx}</div>
                  <div className="grid gap-2 md:grid-cols-2">
                    {lowerRow.map((lower, targetIdx) => (
                      <div
                        key={`sample-${sampleIdx}-target-${targetIdx}`}
                        className="rounded-lg border border-slate-200 bg-slate-50 p-2 text-xs dark:border-slate-800 dark:bg-slate-900/30"
                      >
                        <div className="font-semibold">Target {targetIdx}</div>
                        <div className="mt-1 font-mono">
                          [{lower.toFixed(4)}, {prediction.upper[sampleIdx][targetIdx].toFixed(4)}]
                        </div>
                        <div className="mt-1 text-slate-600 dark:text-slate-400">
                          width={prediction.widths[sampleIdx][targetIdx].toFixed(4)} mid=
                          {prediction.midpoint[sampleIdx][targetIdx].toFixed(4)}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              ))}

              <Divider />
              <div className="grid gap-2 md:grid-cols-2">
                {prediction.intervals.map((interval, idx) => (
                  <KV
                    key={`interval-${idx}`}
                    k={`Mean interval width (target ${idx})`}
                    v={<span className="font-mono">{interval.toFixed(6)}</span>}
                  />
                ))}
              </div>
            </CardBody>
          </Card>
        )}
      </div>

      <div className="col-span-12 xl:col-span-4 space-y-4">
        <Card>
          <CardHeader title="Model Availability" subtitle="Backed by MCCP artifacts on local filesystem." />
          <CardBody className="space-y-2">
            {health ? (
              <>
                <KV
                  k="API status"
                  v={<Chip tone={health.ok ? 'good' : 'bad'}>{health.ok ? 'OK' : 'Error'}</Chip>}
                />
                <KV
                  k="Artifacts"
                  v={
                    <Chip tone={health.model_available ? 'good' : 'warn'}>
                      {health.model_available ? 'Present' : 'Missing'}
                    </Chip>
                  }
                />
                <KV
                  k="Loaded in memory"
                  v={
                    <Chip tone={health.loaded ? 'good' : 'neutral'}>
                      {health.loaded ? 'Loaded' : 'Not loaded'}
                    </Chip>
                  }
                />
                <Divider />
                <label className="block text-xs text-slate-700 dark:text-slate-300">
                  Workspace root
                  <input
                    className="mt-1 h-9 w-full rounded-md border border-slate-300 bg-white px-3 font-mono text-xs dark:border-slate-700 dark:bg-slate-950"
                    value={workspaceRoot}
                    onChange={(e) => {
                      setWorkspaceRoot(e.target.value);
                      setWorkspaceEdited(true);
                      setConfigMsg(null);
                    }}
                    placeholder="/path/to/shared-workspace"
                  />
                </label>
                {configMsg && (
                  <Chip tone={configMsg.includes('Warning') ? 'warn' : 'good'}>{configMsg}</Chip>
                )}
                <Button
                  variant="secondary"
                  onClick={saveConfig}
                  disabled={savingConfig || !workspaceEdited}
                >
                  {savingConfig ? 'Saving...' : 'Update Workspace Path'}
                </Button>
                <div className="text-xs">
                  <div className="text-slate-600 dark:text-slate-400">Model path</div>
                  <div className="font-mono break-all">{health.model_path}</div>
                </div>
                <div className="text-xs">
                  <div className="text-slate-600 dark:text-slate-400">State path</div>
                  <div className="font-mono break-all">{health.state_path}</div>
                </div>
              </>
            ) : (
              <div className="flex items-start gap-2 rounded-lg border border-amber-300/50 bg-amber-50 p-3 text-xs text-amber-900 dark:border-amber-700/50 dark:bg-amber-900/20 dark:text-amber-200">
                <AlertTriangle className="mt-0.5 h-4 w-4 shrink-0" />
                Run a health check first to confirm this tab can reach the MCCP inference service.
              </div>
            )}
          </CardBody>
        </Card>

        <Card>
          <CardHeader title="DevLab Runbook" subtitle="Start local service and wire this tab." />
          <CardBody className="space-y-2 text-xs text-slate-700 dark:text-slate-300">
            <div>1. Start the Python API:</div>
            <pre className="overflow-x-auto rounded-md border border-slate-200 bg-slate-50 p-2 font-mono text-[11px] dark:border-slate-800 dark:bg-slate-900/30">
python ../mccp_inference_devlab_server.py
            </pre>
            <div>2. Keep API URL as <span className="font-mono">http://localhost:8091</span> (or set <span className="font-mono">VITE_MCCP_API_URL</span>).</div>
            <div>3. Paste sample(s) and run inference to inspect interval widths.</div>
          </CardBody>
        </Card>
      </div>
    </div>
  );
}
