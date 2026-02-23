AI Models & Trust
=================

ROBIN uses a PyTorch MLP (``ProcessGeometryMLP``) to predict process geometry
from process parameters, or to suggest parameters for a target geometry.  Models
are managed through the dashboard and the REST API.

Model Control in the Dashboard
------------------------------

Navigate to the **Models & Trust** tab in the sidebar.

.. image:: ../_static/screenshots/dashboard-models.png
   :alt: ROBIN Dashboard - AI Model Control and Trust Management
   :align: center
   :width: 100%

Active Model
~~~~~~~~~~~~

The top card shows the currently loaded checkpoint - name, file path, and size.
The model is loaded at startup from the path specified in the active profile's
``ai.model_path`` field.

Checkpoint Registry
~~~~~~~~~~~~~~~~~~~

All ``.pt`` files found under ``data/models/`` are listed with their size and
modification date.  Click **Load** to switch the active model at runtime.

Quick Prediction
~~~~~~~~~~~~~~~~

Enter the process parameters (e.g. speed, current, voltage), then click
**Predict Geometry** to run a forward pass and see predicted height and width
instantly.

Model Routing
~~~~~~~~~~~~~

ROBIN supports position-based routing: assign different model checkpoints to
robot positions PA and PC.  This lets you run specialised models per station.

Trust Thresholds
~~~~~~~~~~~~~~~~

Set the **Warning** and **Stop** confidence levels.  When the runtime trust
score for a robot drops below a threshold, the corresponding safety gate fires:

* **Warning** - operator is alerted, robot continues
* **Stop** - robot pauses automatically

API Endpoints
-------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Endpoint
     - Description
   * - ``GET /ai/models``
     - List all available model checkpoints
   * - ``GET /ai/models/active``
     - Get the currently loaded model
   * - ``POST /ai/models/select``
     - Load a specific checkpoint by path
   * - ``POST /ai/models/predict``
     - Run a forward prediction with given parameters

Training a New Model
--------------------

.. code-block:: bash

   python scripts/train_profile_model.py

This script:

1. Generates synthetic training data using a physics-inspired simulator
2. Trains a ``ProcessGeometryMLP`` network (configurable hidden layers, dropout)
3. Saves the checkpoint to ``data/models/<profile>/process_geometry_mlp.pt``
   including feature normalization statistics

Each :doc:`profile <profiles>` specifies its own ``ai.model_path``, so training
a new model for one profile does not affect another profile's checkpoint.

Model Architecture
------------------

``ProcessGeometryMLP`` maps 3 input features to 2 outputs:

* **Inputs**: 3 process parameters (order defined by ``ai.feature_order`` in the
  profile YAML - e.g. speed, current, voltage)
* **Outputs**: predicted height, predicted width
* **Normalisation**: per-feature mean/std stored in the checkpoint
* **Configurable**: number of hidden layers, hidden size, dropout rate
