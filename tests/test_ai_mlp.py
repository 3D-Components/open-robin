import torch
from joblib import dump
from sklearn.preprocessing import StandardScaler

from robin.ai import MLPConfig, ProcessGeometryMLP, load_model, save_model


def test_process_geometry_mlp_forward_pass():
    config = MLPConfig(
        input_dim=3,
        hidden_dim=16,
        hidden_layers=1,
        output_dim=2,
        dropout=0.0,
        feature_mean=[1.0, 2.0, 3.0],
        feature_std=[2.0, 2.0, 2.0],
        feature_names=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
    )
    model = ProcessGeometryMLP(config)
    inputs = torch.tensor(
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=torch.float32
    )
    output = model(inputs)
    assert output.shape == (2, 2)


def test_process_geometry_mlp_serialization(tmp_path):
    config = MLPConfig(
        input_dim=3,
        hidden_dim=8,
        hidden_layers=2,
        output_dim=2,
        feature_mean=[0.0, 0.0, 0.0],
        feature_std=[1.0, 1.0, 1.0],
        feature_names=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
    )
    model = ProcessGeometryMLP(config)
    path = tmp_path / 'mlp.pt'
    save_model(model, path)
    loaded = load_model(path)
    assert loaded.config.input_dim == config.input_dim
    assert list(loaded.config.feature_mean) == list(config.feature_mean)
    assert list(loaded.config.feature_names) == list(config.feature_names)


def test_load_model_supports_selected_gold_checkpoint_schema(tmp_path):
    config = MLPConfig(
        input_dim=3,
        hidden_dims=[5, 3],
        output_dim=2,
        dropout=0.0,
        feature_names=(
            'wire_feed_speed_mpm_model_input',
            'travel_speed_mps_model_input',
            'arc_length_correction_mm_model_input',
        ),
    )
    model = ProcessGeometryMLP(config)

    scaler = StandardScaler()
    scaler.fit([[8.0, 0.018, -2.0], [12.0, 0.026, 4.0]])
    scaler_path = tmp_path / 'feature_scaler.joblib'
    dump(scaler, scaler_path)

    selected_gold_state = {
        key.replace('network.', 'net.'): value
        for key, value in model.state_dict().items()
    }
    checkpoint_path = tmp_path / 'selected_gold_model.pt'
    torch.save(
        {
            'schema': 'fragmented_corrected_torch_selected_gold_v1',
            'state_dict': selected_gold_state,
            'feature_columns': list(config.feature_names),
            'hidden_dims': [5, 3],
            'input_dim': 3,
            'output_dim': 2,
            'scaler_path': str(scaler_path),
        },
        checkpoint_path,
    )

    loaded = load_model(checkpoint_path)

    assert list(loaded.config.feature_names) == list(config.feature_names)
    assert list(loaded.config.hidden_dims or []) == [5, 3]
    assert list(loaded.config.feature_mean or []) == list(scaler.mean_)
    assert list(loaded.config.feature_std or []) == list(scaler.scale_)
