import torch

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
        feature_names=('wireSpeed', 'current', 'voltage'),
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
        feature_names=('wireSpeed', 'current', 'voltage'),
    )
    model = ProcessGeometryMLP(config)
    path = tmp_path / 'mlp.pt'
    save_model(model, path)
    loaded = load_model(path)
    assert loaded.config.input_dim == config.input_dim
    assert list(loaded.config.feature_mean) == list(config.feature_mean)
    assert list(loaded.config.feature_names) == list(config.feature_names)
